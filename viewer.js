import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { DragControls } from 'three/examples/jsm/controls/DragControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f172a);
scene.fog = new THREE.Fog(0x0f172a, 10, 500);
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 5, 50);

function updateCameraZoom() {
    if (window.innerWidth < 768) {
        camera.zoom = window.innerWidth / 900;
    } else {
        camera.zoom = 1;
    }
    camera.updateProjectionMatrix();
}
updateCameraZoom();

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.getElementById('canvas-container').appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.maxPolarAngle = Math.PI / 2 - 0.05;

const draggableObjects = [];
const dragControls = new DragControls(draggableObjects, camera, renderer.domElement);
dragControls.transformGroup = true;

dragControls.addEventListener('dragstart', function (event) {
    controls.enabled = false;
    event.object.userData.startPos = event.object.position.clone();
});
dragControls.addEventListener('drag', function (event) {
    event.object.position.z = 0.5;
});
dragControls.addEventListener('dragend', function (event) {
    controls.enabled = true;

    let nearestSlot = null;
    let minDistance = Infinity;
    const objPos = event.object.position;
    validSlots.forEach(slot => {
        let isOccupied = false;
        for (const mesh of loadedMeshes) {
            if (mesh === event.object || mesh.userData.type.startsWith('honeycomb')) continue;

            const targetPos = mesh.userData.targetSnap || mesh.position;
            const mDx = targetPos.x - slot.x;
            const mDy = targetPos.y - slot.y;

            if (mDx * mDx + mDy * mDy < 1.0) {
                isOccupied = true;
                break;
            }
        }

        if (!isOccupied) {
            const dx = slot.x - objPos.x;
            const dy = slot.y - objPos.y;
            const dist = dx * dx + dy * dy;
            if (dist < minDistance) {
                minDistance = dist;
                nearestSlot = slot;
            }
        }
    });

    if (nearestSlot) {
        event.object.userData.targetSnap = new THREE.Vector3(nearestSlot.x, nearestSlot.y, 0.5);
    } else if (event.object.userData.startPos) {
        event.object.userData.targetSnap = event.object.userData.startPos.clone();
    }
});
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
dirLight.position.set(20, 40, 20);
dirLight.castShadow = true;
dirLight.shadow.mapSize.width = 2048;
dirLight.shadow.mapSize.height = 2048;
scene.add(dirLight);

const fillLight = new THREE.DirectionalLight(0x3b82f6, 0.5);
fillLight.position.set(-20, 10, -20);
scene.add(fillLight);

const loader = new STLLoader();
const loadedMeshes = [];
const loadingOverlay = document.createElement('div');
loadingOverlay.id = 'loading-overlay';
loadingOverlay.textContent = 'Loading 3D model...';
document.getElementById('app').appendChild(loadingOverlay);

/*
 * GitHub Pages serves .stl as application/vnd.ms-pki.stl and does not compress
 * it, so a 4.5 MB mesh crosses the wire in full. Every mesh is also stored
 * pre-gzipped; binary STL compresses about 2.7x. Fetch the .gz and inflate it
 * in the browser, falling back to the plain .stl if anything goes wrong.
 */
const canInflate = typeof DecompressionStream !== 'undefined';

async function fetchMesh(path) {
    if (canInflate) {
        try {
            const res = await fetch(path + '.gz');
            if (res.ok) {
                const buffer = await res.arrayBuffer();
                const head = new Uint8Array(buffer, 0, Math.min(2, buffer.byteLength));
                // If a host applied Content-Encoding itself, fetch already inflated this.
                if (head[0] !== 0x1f || head[1] !== 0x8b) return buffer;
                const stream = new Blob([buffer]).stream()
                    .pipeThrough(new DecompressionStream('gzip'));
                return await new Response(stream).arrayBuffer();
            }
        } catch (err) {
            console.warn('Compressed mesh unavailable, falling back to plain STL:', path, err);
        }
    }
    const res = await fetch(path);
    if (!res.ok) throw new Error(`${res.status} ${res.statusText} while loading ${path}`);
    return res.arrayBuffer();
}

// Same signature as STLLoader.load, so existing call sites are unchanged.
loader.load = function (path, onLoad, onProgress, onError) {
    fetchMesh(path)
        .then((buffer) => onLoad(this.parse(buffer)))
        .catch((err) => {
            console.error(err);
            loadingOverlay.style.display = 'none';
            if (onError) onError(err);
            else showError('Could not load that 3D model. Please try again.');
        });
};

const materials = {
    honeycomb: new THREE.MeshStandardMaterial({
        color: 0xffffff,
        roughness: 0.7,
        metalness: 0.1
    }),
    button: new THREE.MeshStandardMaterial({
        color: 0xef4444,
        roughness: 0.3,
        metalness: 0.2
    }),
    lamp: new THREE.MeshStandardMaterial({
        color: 0xf59e0b,
        roughness: 0.1,
        metalness: 0.1,
        transparent: true,
        opacity: 0.95
    }),
    metal: new THREE.MeshStandardMaterial({
        color: 0x9ca3af,
        roughness: 0.4,
        metalness: 0.8
    }),
    black_plastic: new THREE.MeshStandardMaterial({
        color: 0x1f2937,
        roughness: 0.8,
        metalness: 0.1
    }),
    blue_plastic: new THREE.MeshStandardMaterial({
        color: 0x3b82f6,
        roughness: 0.5,
        metalness: 0.1
    }),
    light_grey: new THREE.MeshStandardMaterial({
        color: 0x6b7280,
        roughness: 0.5,
        metalness: 0.3
    })
};

const pieceInfoData = {
    m30: { title: "Thread M30", desc: "Larger threaded fastener. Success requires the bolt to be threaded along its full available length, which demands sustained axial rotation against thread friction.", specs: { "Category": "Precision", "Timeout": "120 s", "Print": "Vertical, 25% infill" } },
    m14: { title: "Thread M14", desc: "Intermediate threaded fastener, available in the library for scene composition. The scored protocol uses the M8 and M30 sizes.", specs: { "Category": "Precision", "Timeout": "Not scored", "Print": "Vertical, 25% infill" } },
    m8: { title: "Thread M8", desc: "The smallest threaded fastener in the library. Its head has to be held against thread friction while turning, which makes it the hardest attachment for every platform reported so far.", specs: { "Category": "Precision", "Timeout": "120 s", "Print": "Vertical, 25% infill" } },
    high_torque_valve: { title: "Gate Valve (Large)", desc: "Gate valve with a larger handwheel, set to a higher required torque. A successful trial rotates the stem one full turn from the closed position, modeled as coupled rotation and translation along the thread axis.", specs: { "Category": "Torque", "Timeout": "120 s", "Print": "Handle up, 30% infill" } },
    small_valve: { title: "Gate Valve (Small)", desc: "Gate valve with a smaller handwheel, scored on one full turn of the stem from the closed position. The narrower rim is the reason one platform failed it while completing the large valve.", specs: { "Category": "Torque", "Timeout": "90 s", "Print": "Handle up, 30% infill" } },
    circuit_breaker: { title: "Circuit Breaker", desc: "Single-pole breaker whose toggle exposes a revolute joint over its physical range. It can be driven by a grasp or by a closed hand structure, and is the fastest attachment on most platforms.", specs: { "Category": "Torque", "Timeout": "60 s", "Print": "25% infill" } },
    ball_valve: { title: "Ball Valve", desc: "Quarter-turn ball valve driven by a lever. Four snap-on friction rings fit over the body, each setting a different rotational friction level, so one printed valve covers several torque settings.", specs: { "Category": "Torque", "Timeout": "60 s / 90 s with ring", "Print": "Handle up, 30% infill" } },
    peg_insertion: { title: "Peg Insertion Plate", desc: "Plate with four identical threaded sockets, three already seated and one free. The 8 mm threaded pin must be aligned with the empty socket and rotated down until it seats.", specs: { "Category": "Precision", "Timeout": "120 s", "Print": "Pegs vertical" } },
    button_composed: { title: "Hidden Push Button", desc: "Push button hidden by a hinged cover. Scored in two stages, opening the cover and pressing the button until actuation, so partial competence is recorded.", specs: { "Category": "Assembly", "Timeout": "60 s", "Stages": "2" } },
    key: { title: "Lock and Key", desc: "Key-and-lock mechanism scored in three stages: grasp the key, insert it vertically, and rotate to unlock. The grasp has to withstand the torque applied at the end of the sequence.", specs: { "Category": "Assembly", "Timeout": "180 s", "Stages": "3" } },
    lamp: { title: "Light Bulb Socket", desc: "Fully printed bulb and socket. The bulb must be threaded until seated, combining continuous rotation about the thread axis with translation along it.", specs: { "Category": "Precision", "Timeout": "120 s", "Print": "Vertical, 25% infill" } },
    drawer: { title: "Sliding Drawer", desc: "Sliding drawer with a recessed handle, scored in three stages: grasp the handle, pull open, push closed. Fingers can enter the recess where parallel jaws cannot.", specs: { "Category": "Assembly", "Timeout": "120 s", "Stages": "3" } },
    shock_absorber: { title: "Shock Absorber", desc: "Two hexagonal end pieces joined by a coil spring with a bolt running through the spring axis. It is the only attachment that occupies two adjacent cells of the board.", specs: { "Category": "Assembly", "Timeout": "180 s", "Stages": "3" } },
    honeycomb_l: { title: "Large Honeycomb Board", desc: "The seven-cell hexagonal base: one central cell surrounded by six outer cells, each an open frame that accepts an attachment through a press-fit interface, with no fasteners required.", specs: { "Category": "Base", "Cells": "7", "Print": "Single piece, 20% infill" } },
    honeycomb_m: { title: "Medium Honeycomb Board", desc: "A reduced version of the honeycomb base for smaller printers and smaller evaluation scenes. It uses the same press-fit interface, so every attachment is interchangeable across board sizes.", specs: { "Category": "Base", "Print": "Single piece, 20% infill" } },
    honeycomb_s: { title: "Small Honeycomb Board", desc: "The most compact honeycomb base, for printers with a limited build volume. Multiple base units can be combined when an attachment occupies more than one cell or a richer scene is needed.", specs: { "Category": "Base", "Print": "Single piece, 20% infill" } }
};

const modelsPaths = {
    honeycomb_l: './models/boards/Honeycomb_Panel.stl',
    honeycomb_m: './models/boards/Honeycomb_Panel_M.stl',
    honeycomb_s: './models/boards/Honeycomb_Panel_S.stl'
};

const CELL_SIZE = 8.65;
const GRID_ANGLE = 30;
let availableSlots = [];
let validSlots = [];
let currentBoardMesh = null;

function updateSlots() {
    const angle = GRID_ANGLE * (Math.PI / 180);
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);

    const rawSlots = [];
    const C = CELL_SIZE;
    const H = CELL_SIZE * 0.86602540378;
    for (let a = -10; a <= 10; a++) {
        for (let b = -10; b <= 10; b++) {
            if (Math.abs(a + b) <= 15) {
                rawSlots.push({
                    x: a * C + b * (C / 2),
                    y: b * H
                });
            }
        }
    }

    rawSlots.sort((A, B) => {
        const distA = A.x * A.x + A.y * A.y;
        const distB = B.x * B.x + B.y * B.y;
        return distA - distB;
    });

    availableSlots = rawSlots.map(s => ({
        x: s.x * cos - s.y * sin,
        y: s.x * sin + s.y * cos
    }));

    validSlots = availableSlots;
}
updateSlots();

let currentSlotIndex = 0;

const compositePieces = {
    m30: [
        { path: './models/torque_based/M30_Screw_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/torque_based/M30_Nut.stl', materialKey: 'light_grey', isBase: false }
    ],
    m14: [
        { path: './models/torque_based/M14_Screw_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/torque_based/M14_Nut.stl', materialKey: 'light_grey', isBase: false }
    ],
    m8: [
        { path: './models/torque_based/M8_Screw_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/torque_based/M8_Nut.stl', materialKey: 'light_grey', isBase: false, endZ: 22 }
    ],
    high_torque_valve: [
        { path: './models/torque_based/High_Torque_Valve_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/torque_based/High_Torque_Valve_Screw.stl', materialKey: 'light_grey', isBase: false, spinOnly: true, turns: 0.5, speed: 0.005, endZ: 125 }
    ],
    small_valve: [
        { path: './models/torque_based/Small_Valve_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/torque_based/Small_Valve_Screw.stl', materialKey: 'light_grey', isBase: false, spinOnly: true, turns: 0.5, speed: 0.005, endZ: 65 }
    ],
    circuit_breaker: [
        { path: './models/precision_based/Circuit_Breaker_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/precision_based/Circuit_Breaker_Switch.stl', materialKey: 'light_grey', isBase: false, spinOnly: true, axis: 'x', turns: 0.15, states: [0, -1, 0, 1], speed: 0.1, endZ: 43, pivotOffset: { x: 0, y: -10, z: -20 } }
    ],
    ball_valve: [
        { path: './models/composed_assembly/Ball_Valve_Base.stl', materialKey: 'honeycomb', isBase: true, fixCenter: { x: 0, y: 0 } },
        { path: './models/composed_assembly/Ball_Valve_Lever.stl', materialKey: 'light_grey', isBase: false, spinOnly: true, turns: -0.25, speed: 0.04, endZ: 50, pivotOffset: { x: -50, y: 0, z: 0 }, positionOffset: { x: 50, y: 0, z: 0 } }
    ],

    peg_insertion: [
        { path: './models/composed_assembly/Shock_Absorber_Base.stl', materialKey: 'honeycomb', isBase: true },
        {
            path: './models/composed_assembly/Shock_Absorber_Pin.stl', materialKey: 'light_grey', isBase: false, speed: 0.04, independentClick: true,
            keyframes: [
                { p: 0, x: 0, y: 0, z: 40 },
                { p: 1, x: 0, y: 0, z: 20 }
            ]
        },
        {
            path: './models/composed_assembly/Shock_Absorber_Pin.stl', materialKey: 'light_grey', isBase: false, speed: 0.04, independentClick: true,
            keyframes: [
                { p: 0, x: 0, y: 28, z: 40 },
                { p: 1, x: 0, y: 28, z: 20 }
            ]
        },
        {
            path: './models/composed_assembly/Shock_Absorber_Pin.stl', materialKey: 'light_grey', isBase: false, speed: 0.04, independentClick: true,
            keyframes: [
                { p: 0, x: -35, y: 0, z: 40 },
                { p: 1, x: -35, y: 0, z: 20 }
            ]
        },
        {
            path: './models/composed_assembly/Shock_Absorber_Pin.stl', materialKey: 'light_grey', isBase: false, speed: 0.04, independentClick: true,
            keyframes: [
                { p: 0, x: 25, y: 0, z: 40 },
                { p: 1, x: 25, y: 0, z: 20 }
            ]
        }
    ],
    button_composed: [
        { path: './models/precision_based/Button_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/precision_based/Button.stl', materialKey: 'button', isBase: false, turns: 0, startZ: 22, endZ: 17, speed: 0.08, states: [0, 0, 1, 0] },
        { path: './models/composed_assembly/Button_Cover.stl', materialKey: 'light_grey', isBase: false, spinOnly: true, axis: 'x', turns: -0.3, speed: 0.08, pivotOffset: { x: 0, y: 32, z: -4 }, endZ: 22, states: [0, 1, 1, 1] }
    ],
    key: [
        { path: './models/composed_assembly/Key_Base.stl', materialKey: 'honeycomb', isBase: true, mirrorX: true },
        {
            path: './models/composed_assembly/Key.stl', materialKey: 'metal', isBase: false, fixRotation: { x: Math.PI / 2 }, speed: 0.02, pivotOffset: { x: -4, y: 0, z: 0 },
            keyframes: [
                { p: 0, x: 0, z: 60, rz: 0 },
                { p: 0.5, x: 0, z: 18, rz: 0 },
                { p: 1, x: 5, z: 18, rz: -Math.PI / 2 }
            ]
        }
    ],
    lamp: [
        { path: './models/composed_assembly/Lamp_Base.stl', materialKey: 'honeycomb', isBase: true },
        { path: './models/composed_assembly/Half_Lamp.stl', materialKey: 'lamp', isBase: false, turns: 2, startZ: 50, endZ: 25, speed: 0.03, keepCADOrigin: true },
        { path: './models/composed_assembly/Half_Lamp.stl', materialKey: 'lamp', isBase: false, turns: 2, startZ: 50, endZ: 25, speed: 0.03, fixRotation: { z: Math.PI }, keepCADOrigin: true }
    ],
    drawer: [
        { path: './models/composed_assembly/Drawer_Base.stl', materialKey: 'honeycomb', isBase: true },
        {
            path: './models/composed_assembly/Drawer.stl', materialKey: 'blue_plastic', isBase: false, turns: 0, speed: 0.005, fixRotation: { z: Math.PI },
            keyframes: [
                { p: 0, x: -12, z: 65 },
                { p: 0.33, x: -12, z: 50 },
                { p: 0.66, x: 0, z: 50 },
                { p: 1, x: 0, z: 30 }
            ]
        }
    ],
    shock_absorber_base1: [
        { path: './models/composed_assembly/Shock_Absorber_Base.stl', materialKey: 'honeycomb', isBase: true },
        {
            path: './models/composed_assembly/Shock_Absorber_Screw.stl', materialKey: 'light_grey', isBase: false, speed: 0.04, independentClick: true,
            hasHoles: true, currentHole: 0, holes: [{ x: 0, y: 0 }, { x: 0, y: 28 }, { x: -15, y: -29 }, { x: -35, y: 0 }, { x: 25, y: 0 }],
            keyframes: [
                { p: 0, x: 0, y: 0, z: 40 },
                { p: 1, x: 0, y: 0, z: 30 }
            ]
        }
    ],
    shock_absorber_base2: [
        { path: './models/composed_assembly/Shock_Absorber_Base.stl', materialKey: 'honeycomb', isBase: true },
        {
            path: './models/composed_assembly/Shock_Absorber_Screw.stl', materialKey: 'light_grey', isBase: false, speed: 0.04, independentClick: true,
            hasHoles: true, currentHole: 0, holes: [{ x: 0, y: 0 }, { x: 0, y: 28 }, { x: -15, y: -29 }, { x: -35, y: 0 }, { x: 25, y: 0 }],
            keyframes: [
                { p: 0, x: 0, y: 0, z: 40 },
                { p: 1, x: 0, y: 0, z: 30 }
            ]
        }
    ],
    shock_absorber_spring: [
        { path: './models/composed_assembly/Shock_Absorber.stl', materialKey: 'light_grey', isBase: false, isSpring: true, endZ: 50 }
    ]
};

function loadPiece(type, linkId = null) {
    if (type === 'shock_absorber') {
        const largePieces = ['high_torque_valve', 'small_valve', 'ball_valve', 'shock_absorber_base1', 'shock_absorber_base2'];
        const hasLargePiece = loadedMeshes.some(m => largePieces.includes(m.userData.type));
        if (hasLargePiece) {
            showError("Only one large piece can be placed at a time to prevent collisions. Remove the current one first.");
            return;
        }

        const newLinkId = 'shock_' + Date.now();
        loadPiece('shock_absorber_base1', newLinkId);
        const skip = Math.max(2, Math.floor(validSlots.length / 6));
        currentSlotIndex = (currentSlotIndex + skip) % validSlots.length;
        loadPiece('shock_absorber_base2', newLinkId);
        loadPiece('shock_absorber_spring', newLinkId);
        return;
    }

    loadingOverlay.style.display = 'block';

    if (type.startsWith('honeycomb')) {
        const path = modelsPaths[type];
        loader.load(path, (geometry) => {
            geometry.computeVertexNormals();

            const material = materials['honeycomb'];
            const mesh = new THREE.Mesh(geometry, material);
            const scaleFactor = 0.1;
            mesh.scale.set(scaleFactor, scaleFactor, scaleFactor);
            mesh.castShadow = false;
            mesh.receiveShadow = false;
            mesh.position.set(0, 0, 0);

            for (let i = loadedMeshes.length - 1; i >= 0; i--) {
                const oldMesh = loadedMeshes[i];
                scene.remove(oldMesh);

                if (oldMesh.isGroup) {
                    oldMesh.children[0].children.forEach(c => {
                        if (c.geometry) c.geometry.dispose();
                        if (c.material) {
                            if (Array.isArray(c.material)) c.material.forEach(m => m.dispose());
                            else c.material.dispose();
                        }
                    });
                } else {
                    if (oldMesh.geometry) oldMesh.geometry.dispose();
                    if (oldMesh.material) {
                        if (Array.isArray(oldMesh.material)) oldMesh.material.forEach(m => m.dispose());
                        else oldMesh.material.dispose();
                    }
                }
            }
            loadedMeshes.length = 0;
            draggableObjects.length = 0;

            mesh.userData.type = type;
            scene.add(mesh);
            loadedMeshes.push(mesh);

            currentBoardMesh = mesh;
            currentSlotIndex = 0;

            const bbox = new THREE.Box3().setFromObject(mesh);
            const margin = 3;

            validSlots = availableSlots.filter(slot => {
                return slot.x > bbox.min.x + margin && slot.x < bbox.max.x - margin &&
                    slot.y > bbox.min.y + margin && slot.y < bbox.max.y - margin;
            });

            loadingOverlay.style.display = 'none';
        });
    } else {
        const largePieces = ['high_torque_valve', 'small_valve', 'ball_valve', 'shock_absorber_base1', 'shock_absorber_base2'];
        if (largePieces.includes(type)) {
            const hasLargePiece = loadedMeshes.some(m => {
                if (!largePieces.includes(m.userData.type)) return false;
                if (type === 'shock_absorber_base2' && m.userData.type === 'shock_absorber_base1') return false;
                return true;
            });
            if (hasLargePiece) {
                showError("Only one large piece can be placed at a time to prevent collisions. Remove the current one first.");
                loadingOverlay.style.display = 'none';
                return;
            }
        }

        const parts = compositePieces[type];
        const isSpring = parts.some(p => p.isSpring);
        const innerGroup = new THREE.Group();
        const wrapper = new THREE.Group();
        let loadedCount = 0;
        let baseCenter = new THREE.Vector3();

        let slot = null;
        if (!isSpring) {
            for (let i = 0; i < validSlots.length; i++) {
                const index = (currentSlotIndex + i) % validSlots.length;
                const testSlot = validSlots[index];

                let isOccupied = false;
                for (const mesh of loadedMeshes) {
                    if (mesh.userData.type.startsWith('honeycomb')) continue;

                    const targetPos = mesh.userData.targetSnap || mesh.position;
                    const dx = targetPos.x - testSlot.x;
                    const dy = targetPos.y - testSlot.y;
                    if (dx * dx + dy * dy < 1.0) {
                        isOccupied = true;
                        break;
                    }
                }

                if (!isOccupied) {
                    slot = testSlot;
                    currentSlotIndex = index + 1;
                    break;
                }
            }

            if (!slot) {
                showError("The board is full! Clear the scene or change to a larger board to add more pieces.");
                loadingOverlay.style.display = 'none';
                return;
            }
        }

        parts.forEach(part => {
            loader.load(part.path, (geometry) => {
                if (part.fixRotation) {
                    if (part.fixRotation.x) geometry.rotateX(part.fixRotation.x);
                    if (part.fixRotation.y) geometry.rotateY(part.fixRotation.y);
                    if (part.fixRotation.z) geometry.rotateZ(part.fixRotation.z);
                }

                if (part.mirrorX || part.mirrorY) {
                    if (part.mirrorX) geometry.scale(-1, 1, 1);
                    if (part.mirrorY) geometry.scale(1, -1, 1);

                    const positions = geometry.attributes.position;
                    for (let i = 0; i < positions.count; i += 3) {
                        const x1 = positions.getX(i + 1);
                        const y1 = positions.getY(i + 1);
                        const z1 = positions.getZ(i + 1);

                        const x2 = positions.getX(i + 2);
                        const y2 = positions.getY(i + 2);
                        const z2 = positions.getZ(i + 2);

                        positions.setXYZ(i + 1, x2, y2, z2);
                        positions.setXYZ(i + 2, x1, y1, z1);
                    }
                    geometry.computeVertexNormals();
                }

                if (part.isSpring) {
                    geometry.computeBoundingBox();
                    const center = new THREE.Vector3();
                    geometry.boundingBox.getCenter(center);
                    geometry.translate(-center.x, -center.y, -center.z);
                }

                if (part.isBase) {
                    geometry.computeBoundingBox();
                    geometry.boundingBox.getCenter(baseCenter);
                    baseCenter.z = 0;

                    if (part.fixCenter) {
                        baseCenter.x += part.fixCenter.x;
                        baseCenter.y += part.fixCenter.y;
                    }
                }

                const material = materials[part.materialKey] || new THREE.MeshStandardMaterial({ color: 0xcccccc });
                const mesh = new THREE.Mesh(geometry, material.clone());
                mesh.castShadow = true;
                mesh.receiveShadow = true;

                if (part.isSpring) {
                    const originalPositions = new Float32Array(geometry.attributes.position.array);
                    geometry.setAttribute('originalPosition', new THREE.BufferAttribute(originalPositions, 3));
                    mesh.userData.isDynamicSpring = true;
                }

                mesh.userData.isBase = part.isBase;
                mesh.userData.partConfig = part;
                innerGroup.add(mesh);
                loadedCount++;

                if (loadedCount === parts.length) {
                    innerGroup.children.forEach(child => {
                        if (child.userData.isBase) {
                            child.position.sub(baseCenter);
                            if (child.userData.partConfig.positionOffset) {
                                child.position.x += child.userData.partConfig.positionOffset.x || 0;
                                child.position.y += child.userData.partConfig.positionOffset.y || 0;
                            }
                        } else {
                            child.geometry.computeBoundingBox();
                            const objCenter = new THREE.Vector3();
                            child.geometry.boundingBox.getCenter(objCenter);

                            const config = child.userData.partConfig;

                            if (config.keepCADOrigin) {
                                child.geometry.translate(-baseCenter.x, -baseCenter.y, -baseCenter.z);
                            } else {
                                child.geometry.translate(-objCenter.x, -objCenter.y, -objCenter.z);
                            }

                            let pivotX = 0, pivotY = 0, pivotZ = 0;
                            if (config.pivotOffset) {
                                child.geometry.translate(-config.pivotOffset.x, -config.pivotOffset.y, -config.pivotOffset.z);
                                pivotX = config.pivotOffset.x;
                                pivotY = config.pivotOffset.y;
                                pivotZ = config.pivotOffset.z;
                            }

                            child.position.x = pivotX;
                            child.position.y = pivotY;

                            if (config.positionOffset) {
                                child.position.x += config.positionOffset.x || 0;
                                child.position.y += config.positionOffset.y || 0;
                            }

                            const finalDepth = config.endZ !== undefined ? config.endZ : 18;
                            const endZ = finalDepth + pivotZ;

                            const startZOffset = config.startZ !== undefined ? config.startZ : 50;
                            const startZ = config.spinOnly ? endZ : (startZOffset + pivotZ);

                            child.position.z = startZ;

                            child.userData.startZ = startZ;
                            child.userData.endZ = endZ;
                            child.userData.progress = 0;
                            child.userData.targetProgress = 0;
                            child.userData.startTurns = config.startTurns !== undefined ? config.startTurns : 0;
                            child.userData.turns = config.turns !== undefined ? config.turns : 0.5;
                            child.userData.speed = config.speed !== undefined ? config.speed : 0.02;
                            child.userData.axis = config.axis || 'z';

                            if (config.keyframes) {
                                child.userData.keyframes = config.keyframes;
                                if (config.keyframes[0].x !== undefined) child.position.x = config.keyframes[0].x;
                                if (config.keyframes[0].y !== undefined) child.position.y = config.keyframes[0].y;
                                if (config.keyframes[0].z !== undefined) child.position.z = config.keyframes[0].z;
                                if (config.keyframes[0].rx !== undefined) child.rotation.x = config.keyframes[0].rx;
                                if (config.keyframes[0].ry !== undefined) child.rotation.y = config.keyframes[0].ry;
                                if (config.keyframes[0].rz !== undefined) child.rotation.z = config.keyframes[0].rz;
                            }

                            if (config.states) {
                                child.userData.states = config.states;
                                child.userData.currentState = 0;
                                child.userData.targetProgress = config.states[0];
                                child.userData.progress = config.states[0];
                            }
                        }
                    });

                    const scaleFactor = 0.1;
                    innerGroup.scale.set(scaleFactor, scaleFactor, scaleFactor);

                    wrapper.add(innerGroup);
                    if (!isSpring) {
                        wrapper.position.x = slot.x;
                        wrapper.position.y = slot.y;
                    }
                    wrapper.position.z = 0.5;

                    wrapper.userData.type = type;
                    if (linkId) {
                        wrapper.userData.linkId = linkId;
                    }
                    scene.add(wrapper);
                    loadedMeshes.push(wrapper);

                    if (!parts[0].unselectable && !isSpring) {
                        draggableObjects.push(wrapper);
                    }

                    loadingOverlay.style.display = 'none';
                }
            });
        });
    }
}

document.querySelectorAll('.add-piece').forEach(btn => {
    btn.addEventListener('click', (e) => {
        const type = e.target.getAttribute('data-type');
        loadPiece(type);
    });
});

document.querySelectorAll('.change-board').forEach(btn => {
    btn.addEventListener('click', (e) => {
        const type = e.target.getAttribute('data-type');
        loadPiece(type);
    });
});

document.getElementById('reset-scene').addEventListener('click', () => {
    for (let i = loadedMeshes.length - 1; i >= 0; i--) {
        const mesh = loadedMeshes[i];
        if (!mesh.userData.type.startsWith('honeycomb')) {
            scene.remove(mesh);
            if (mesh.isGroup) {
                mesh.children[0].children.forEach(c => {
                    if (c.geometry) c.geometry.dispose();
                    if (c.material) {
                        if (Array.isArray(c.material)) c.material.forEach(m => m.dispose());
                        else c.material.dispose();
                    }
                });
            } else {
                if (mesh.geometry) mesh.geometry.dispose();
                if (mesh.material) {
                    if (Array.isArray(mesh.material)) mesh.material.forEach(m => m.dispose());
                    else mesh.material.dispose();
                }
            }
            loadedMeshes.splice(i, 1);

            const dragIndex = draggableObjects.indexOf(mesh);
            if (dragIndex > -1) {
                draggableObjects.splice(dragIndex, 1);
            }
        }
    }
    currentSlotIndex = 0;

    selectedObject = null;
    const deleteBtn = document.getElementById('delete-btn');
    if (deleteBtn) deleteBtn.style.display = 'none';
    const rotateBtn = document.getElementById('rotate-btn');
    if (rotateBtn) rotateBtn.style.display = 'none';
    const movePinBtn = document.getElementById('move-pin-btn');
    if (movePinBtn) movePinBtn.style.display = 'none';
    const infoPanel = document.getElementById('piece-info-panel');
    if (infoPanel) infoPanel.classList.add('hidden');
});

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
let selectedObject = null;
const deleteBtn = document.getElementById('delete-btn');
const rotateBtn = document.getElementById('rotate-btn');

renderer.domElement.addEventListener('pointerdown', (event) => {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);

    const infoPanel = document.getElementById('piece-info-panel');

    if (selectedObject) {
        if (selectedObject.isGroup) {
            const objsToRevert = [selectedObject];
            if (selectedObject.userData.linkId) {
                loadedMeshes.forEach(m => {
                    if (m !== selectedObject && m.userData.linkId === selectedObject.userData.linkId) {
                        objsToRevert.push(m);
                    }
                });
            }
            objsToRevert.forEach(obj => {
                obj.children[0].children.forEach(mesh => {
                    if (mesh.userData.originalMaterial) {
                        mesh.material = mesh.userData.originalMaterial;
                    }
                });
            });
        }
        selectedObject = null;
        if (deleteBtn) deleteBtn.style.display = 'none';
        if (rotateBtn) rotateBtn.style.display = 'none';
        const movePinBtn = document.getElementById('move-pin-btn');
        if (movePinBtn) movePinBtn.style.display = 'none';
        if (infoPanel) infoPanel.classList.add('hidden');
    }

    const intersects = raycaster.intersectObjects(loadedMeshes, true);

    if (intersects.length > 0) {
        let object = intersects[0].object;
        while (object.parent && !loadedMeshes.includes(object)) {
            object = object.parent;
        }

        if (loadedMeshes.includes(object)) {
            selectedObject = object;

            if (object.isGroup) {
                const clickedMesh = intersects[0].object;
                const isIndependent = clickedMesh.userData.partConfig && clickedMesh.userData.partConfig.independentClick;

                const meshesToAnimate = isIndependent ? [clickedMesh] : selectedObject.children[0].children;

                meshesToAnimate.forEach(mesh => {
                    if (!mesh.userData.isBase && mesh.userData.startZ !== undefined) {
                        if (mesh.userData.states) {
                            mesh.userData.currentState = (mesh.userData.currentState + 1) % mesh.userData.states.length;
                            mesh.userData.targetProgress = mesh.userData.states[mesh.userData.currentState];
                        } else {
                            mesh.userData.targetProgress = mesh.userData.targetProgress === 1 ? 0 : 1;
                        }
                    }
                });

                const objsToColor = [selectedObject];
                if (selectedObject.userData.linkId) {
                    loadedMeshes.forEach(m => {
                        if (m !== selectedObject && m.userData.linkId === selectedObject.userData.linkId) {
                            objsToColor.push(m);
                        }
                    });
                }

                let showMoveBtn = false;

                objsToColor.forEach(obj => {
                    obj.children[0].children.forEach(mesh => {
                        if (!mesh.userData.originalMaterial) {
                            mesh.userData.originalMaterial = mesh.material;
                        }
                        mesh.material = mesh.userData.originalMaterial.clone();
                        if (mesh.material.emissive) {
                            mesh.material.emissive.setHex(0x3b82f6);
                            mesh.material.emissiveIntensity = 0.1;
                        }
                        if (mesh.userData.partConfig && mesh.userData.partConfig.hasHoles) {
                            showMoveBtn = true;
                        }
                    });
                });

                if (deleteBtn) deleteBtn.style.display = 'flex';
                if (rotateBtn) rotateBtn.style.display = 'flex';
                const movePinBtn = document.getElementById('move-pin-btn');
                if (movePinBtn) {
                    movePinBtn.style.display = showMoveBtn ? 'flex' : 'none';
                }
            }

            const typeStr = object.userData.type.startsWith('shock_absorber') ? 'shock_absorber' : object.userData.type;
            const infoData = pieceInfoData[typeStr];
            if (infoPanel && infoData) {
                document.getElementById('info-title').textContent = infoData.title;
                document.getElementById('info-desc').textContent = infoData.desc;
                const specsList = document.getElementById('info-specs');
                specsList.innerHTML = '';
                for (const [key, value] of Object.entries(infoData.specs)) {
                    specsList.innerHTML += `<li><span>${key}</span> <span>${value}</span></li>`;
                }

                if (window.innerWidth <= 768) {
                    infoPanel.classList.add('collapsed');
                } else {
                    infoPanel.classList.remove('collapsed');
                }

                infoPanel.classList.remove('hidden');
            }
        }
    }
});

const viewToggleBtn = document.getElementById('view-toggle-btn');
if (viewToggleBtn) {
    viewToggleBtn.addEventListener('click', () => {
        const bottomBar = document.getElementById('bottom-bar');
        if (bottomBar) {
            bottomBar.classList.toggle('collapsed-views');
        }
    });
}

const infoHeader = document.getElementById('info-header');
if (infoHeader) {
    infoHeader.addEventListener('click', () => {
        document.getElementById('piece-info-panel').classList.toggle('collapsed');
    });
}

if (deleteBtn) {
    deleteBtn.addEventListener('click', () => {
        if (selectedObject) {
            const objsToDelete = [selectedObject];

            if (selectedObject.userData.linkId) {
                loadedMeshes.forEach(mesh => {
                    if (mesh !== selectedObject && mesh.userData.linkId === selectedObject.userData.linkId) {
                        objsToDelete.push(mesh);
                    }
                });
            }

            objsToDelete.forEach(obj => {
                scene.remove(obj);
                const indexM = loadedMeshes.indexOf(obj);
                if (indexM > -1) loadedMeshes.splice(indexM, 1);

                const indexD = draggableObjects.indexOf(obj);
                if (indexD > -1) draggableObjects.splice(indexD, 1);

                if (obj.isGroup) {
                    obj.children[0].children.forEach(c => {
                        if (c.geometry) c.geometry.dispose();
                        if (c.material) {
                            if (Array.isArray(c.material)) c.material.forEach(m => m.dispose());
                            else c.material.dispose();
                        }
                    });
                } else {
                    if (obj.geometry) obj.geometry.dispose();
                    if (obj.material) {
                        if (Array.isArray(obj.material)) obj.material.forEach(m => m.dispose());
                        else obj.material.dispose();
                    }
                }
            });

            selectedObject = null;
            deleteBtn.style.display = 'none';
            if (rotateBtn) rotateBtn.style.display = 'none';
            const movePinBtn = document.getElementById('move-pin-btn');
            if (movePinBtn) movePinBtn.style.display = 'none';
        }
    });
}

if (rotateBtn) {
    rotateBtn.addEventListener('click', () => {
        if (selectedObject) {
            selectedObject.rotation.z += Math.PI / 3;
        }
    });
}

const movePinBtn = document.getElementById('move-pin-btn');
if (movePinBtn) {
    movePinBtn.addEventListener('click', () => {
        if (selectedObject) {
            selectedObject.children[0].children.forEach(mesh => {
                if (mesh.userData.partConfig && mesh.userData.partConfig.hasHoles) {
                    const conf = mesh.userData.partConfig;
                    if (conf.currentHole === undefined) conf.currentHole = 0;
                    conf.currentHole = (conf.currentHole + 1) % conf.holes.length;
                    const newHole = conf.holes[conf.currentHole];

                    mesh.userData.keyframes = [
                        { p: 0, x: newHole.x, y: newHole.y, z: 40 },
                        { p: 1, x: newHole.x, y: newHole.y, z: 30 }
                    ];

                    mesh.position.x = newHole.x;
                    mesh.position.y = newHole.y;
                }
            });
        }
    });
}

const mobileMenuBtn = document.getElementById('mobile-menu-btn');
const controlsPanel = document.querySelector('.controls');

controlsPanel.classList.add('mobile-hidden');

if (mobileMenuBtn) {
    mobileMenuBtn.addEventListener('click', () => {
        controlsPanel.classList.toggle('mobile-hidden');
    });
}

document.querySelectorAll('.add-piece, .change-board').forEach(btn => {
    btn.addEventListener('click', () => {
        controlsPanel.classList.add('mobile-hidden');
    });
});


document.addEventListener('pointerdown', (event) => {
    if (!controlsPanel.classList.contains('mobile-hidden')) {
        if (!controlsPanel.contains(event.target) && !mobileMenuBtn.contains(event.target)) {
            controlsPanel.classList.add('mobile-hidden');
        }
    }
    const bottomBar = document.getElementById('bottom-bar');
    if (bottomBar && !bottomBar.classList.contains('collapsed-views')) {
        if (!bottomBar.contains(event.target)) {
            bottomBar.classList.add('collapsed-views');
        }
    }
});

let isCentering = false;
const targetCameraPos = new THREE.Vector3(0, 5, 50);
const targetControlPos = new THREE.Vector3(0, 0, 0);

document.getElementById('center-view').addEventListener('click', () => {
    targetCameraPos.set(0, 5, 50);
    targetControlPos.set(0, 0, 0);
    isCentering = true;
    document.getElementById('bottom-bar').classList.add('collapsed-views');
});

document.getElementById('view-top').addEventListener('click', () => {
    targetCameraPos.set(0, 60, 0.1);
    targetControlPos.set(0, 0, 0);
    isCentering = true;
    document.getElementById('bottom-bar').classList.add('collapsed-views');
});

document.getElementById('view-side').addEventListener('click', () => {
    targetCameraPos.set(50, 5, 0);
    targetControlPos.set(0, 0, 0);
    isCentering = true;
    document.getElementById('bottom-bar').classList.add('collapsed-views');
});

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    updateCameraZoom();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

const clock = new THREE.Clock();

// Scratch objects reused across frames. Allocating these inside animate() meant
// nine new THREE instances per frame per shock absorber, which showed up as
// periodic GC stutter while dragging.
const _pos1 = new THREE.Vector3();
const _pos2 = new THREE.Vector3();
const _mid = new THREE.Vector3();
const _direction = new THREE.Vector3();
const _up = new THREE.Vector3(0, 0, 1);
const _xAxis = new THREE.Vector3();
const _yAxis = new THREE.Vector3();
const _basis = new THREE.Matrix4();
const _size = new THREE.Vector3();
const _springLinks = new Map();

const appEl = document.getElementById('app');

function animate() {
    requestAnimationFrame(animate);

    if (appEl && appEl.classList.contains('hidden-app')) return;

    const time = clock.getElapsedTime();

    if (isCentering) {
        camera.position.lerp(targetCameraPos, 0.015);
        controls.target.lerp(targetControlPos, 0.015);

        if (camera.position.distanceTo(targetCameraPos) < 0.1 && controls.target.distanceTo(targetControlPos) < 0.1) {
            isCentering = false;
            camera.position.copy(targetCameraPos);
            controls.target.copy(targetControlPos);
        }
    }

    // Resolve every shock-absorber base in one pass instead of scanning
    // loadedMeshes twice per spring (which made this quadratic).
    _springLinks.clear();
    for (const m of loadedMeshes) {
        const t = m.userData.type;
        if (t === 'shock_absorber_base1' || t === 'shock_absorber_base2') {
            let entry = _springLinks.get(m.userData.linkId);
            if (!entry) _springLinks.set(m.userData.linkId, entry = {});
            entry[t === 'shock_absorber_base1' ? 'base1' : 'base2'] = m;
        }
    }

    loadedMeshes.forEach(wrapper => {
        if (wrapper.userData.type === 'shock_absorber_spring') {
            const link = _springLinks.get(wrapper.userData.linkId);
            const base1 = link && link.base1;
            const base2 = link && link.base2;

            if (base1 && base2 && wrapper.children.length > 0 && wrapper.children[0].children.length > 0) {
                const screw1 = base1.children[0].children.find(m => m.userData.partConfig && m.userData.partConfig.hasHoles);
                const screw2 = base2.children[0].children.find(m => m.userData.partConfig && m.userData.partConfig.hasHoles);

                if (screw1 && screw2) {
                    const pos1 = _pos1;
                    const pos2 = _pos2;
                    screw1.getWorldPosition(pos1);
                    screw2.getWorldPosition(pos2);

                    if (screw1.userData.heightOffset === undefined) {
                        screw1.geometry.computeBoundingBox();
                        screw1.userData.heightOffset = screw1.geometry.boundingBox.max.z * 0.1;
                    }
                    if (screw2.userData.heightOffset === undefined) {
                        screw2.geometry.computeBoundingBox();
                        screw2.userData.heightOffset = screw2.geometry.boundingBox.max.z * 0.1;
                    }

                    pos1.z += screw1.userData.heightOffset * 0.3;
                    pos2.z += screw2.userData.heightOffset * 0.3;

                    const distance = pos1.distanceTo(pos2);
                    const mid = _mid.addVectors(pos1, pos2).multiplyScalar(0.5);
                    wrapper.position.copy(mid);

                    const direction = _direction.subVectors(pos2, pos1).normalize();
                    const up = _up;
                    const xAxis = _xAxis.crossVectors(up, direction);
                    if (xAxis.lengthSq() < 0.001) {
                        xAxis.set(1, 0, 0);
                    } else {
                        xAxis.normalize();
                    }
                    const yAxis = _yAxis.crossVectors(direction, xAxis).normalize();
                    const m = _basis;
                    m.makeBasis(xAxis, yAxis, direction);
                    wrapper.quaternion.setFromRotationMatrix(m);

                    const springMesh = wrapper.children[0].children[0];
                    if (!wrapper.userData.springLen) {
                        springMesh.geometry.computeBoundingBox();
                        const size = _size;
                        springMesh.geometry.boundingBox.getSize(size);

                        if (size.x > size.y && size.x > size.z) {
                            wrapper.userData.springLen = size.x * 0.1;
                            wrapper.userData.springAxis = 'x';
                        } else if (size.y > size.x && size.y > size.z) {
                            wrapper.userData.springLen = size.y * 0.1;
                            wrapper.userData.springAxis = 'y';
                        } else {
                            wrapper.userData.springLen = size.z * 0.1;
                            wrapper.userData.springAxis = 'z';
                        }
                    }

                    const len = wrapper.userData.springLen || 1;
                    const scaleZ = (distance / len) * 1.16;

                    springMesh.position.set(0, 0, 0);

                    if (springMesh.userData.isDynamicSpring) {
                        const uStretch = Math.max(0.1, (distance * 5.0 - 19.0) / 22.0);

                        if (Math.abs((springMesh.userData.lastStretch || 1) - uStretch) > 0.001) {
                            springMesh.userData.lastStretch = uStretch;
                            const posAttr = springMesh.geometry.attributes.position;
                            const origAttr = springMesh.geometry.attributes.originalPosition;
                            for (let i = 0; i < posAttr.count; i++) {
                                const z = origAttr.getZ(i);
                                const absZ = Math.abs(z);
                                if (absZ > 22.0) {
                                    const signZ = Math.sign(z);
                                    const stretchOffset = 22.0 * uStretch;
                                    const hookOffset = absZ - 22.0;
                                    posAttr.setZ(i, signZ * (stretchOffset + hookOffset));
                                } else {
                                    posAttr.setZ(i, z * uStretch);
                                }
                            }
                            posAttr.needsUpdate = true;
                            springMesh.geometry.computeVertexNormals();
                        }
                        wrapper.scale.set(1, 1, 1);
                    } else {
                        wrapper.scale.set(1, 1, scaleZ);
                    }

                    if (wrapper.userData.springAxis === 'z') {
                        springMesh.rotation.z = Math.PI / 2;
                    } else if (wrapper.userData.springAxis === 'x') {
                        springMesh.rotation.y = -Math.PI / 2;
                        springMesh.rotation.x = Math.PI / 2;
                    } else if (wrapper.userData.springAxis === 'y') {
                        springMesh.rotation.x = Math.PI / 2;
                        springMesh.rotation.y = Math.PI / 2;
                    }
                }
            }
        }

        if (wrapper.userData.targetSnap) {
            wrapper.position.lerp(wrapper.userData.targetSnap, 0.15);
            if (wrapper.position.distanceTo(wrapper.userData.targetSnap) < 0.05) {
                wrapper.position.copy(wrapper.userData.targetSnap);
                wrapper.userData.targetSnap = null;
            }
        }

        if (!wrapper.userData.type.startsWith('honeycomb') && wrapper.userData.type !== 'shock_absorber_spring') {
            wrapper.children[0].children.forEach(mesh => {
                if (!mesh.userData.isBase && mesh.userData.startZ !== undefined) {
                    const spd = mesh.userData.speed || 0.02;
                    const diff = mesh.userData.targetProgress - mesh.userData.progress;
                    mesh.userData.progress += diff * spd;

                    if (Math.abs(diff) > 0.001) {

                        if (mesh.userData.keyframes) {
                            const keys = mesh.userData.keyframes;
                            const p = mesh.userData.progress;
                            let k1 = keys[0], k2 = keys[keys.length - 1];

                            for (let i = 0; i < keys.length - 1; i++) {
                                if (p >= keys[i].p && p <= keys[i + 1].p) {
                                    k1 = keys[i];
                                    k2 = keys[i + 1];
                                    break;
                                }
                            }

                            const interval = k2.p - k1.p;
                            const t = interval === 0 ? 0 : (p - k1.p) / interval;

                            if (k1.x !== undefined) mesh.position.x = THREE.MathUtils.lerp(k1.x, k2.x, t);
                            if (k1.y !== undefined) mesh.position.y = THREE.MathUtils.lerp(k1.y, k2.y, t);
                            if (k1.z !== undefined) mesh.position.z = THREE.MathUtils.lerp(k1.z, k2.z, t);

                            if (k1.rx !== undefined) mesh.rotation.x = THREE.MathUtils.lerp(k1.rx, k2.rx, t);
                            if (k1.ry !== undefined) mesh.rotation.y = THREE.MathUtils.lerp(k1.ry, k2.ry, t);
                            if (k1.rz !== undefined) mesh.rotation.z = THREE.MathUtils.lerp(k1.rz, k2.rz, t);

                        } else {
                            mesh.position.z = THREE.MathUtils.lerp(mesh.userData.startZ, mesh.userData.endZ, mesh.userData.progress);

                            const turns = mesh.userData.turns;
                            const startTurns = mesh.userData.startTurns;
                            const angulo = THREE.MathUtils.lerp(Math.PI * 2 * startTurns, Math.PI * 2 * turns, mesh.userData.progress);

                            if (mesh.userData.axis === 'x') {
                                mesh.rotation.x = angulo;
                            } else if (mesh.userData.axis === 'y') {
                                mesh.rotation.y = angulo;
                            } else {
                                mesh.rotation.z = angulo;
                            }
                        }

                        if (mesh.userData.partConfig.materialKey === 'lamp') {
                            if (mesh.userData.progress >= 0.95) {
                                mesh.material.emissive.setHex(0xfff5b5);
                                mesh.material.emissiveIntensity = 0.8;
                            } else {
                                mesh.material.emissiveIntensity = 0;
                            }
                        }
                    }
                }
            });
        }
    });

    controls.update();
    renderer.render(scene, camera);
}

let toastTimeout;
function showError(message) {
    const toast = document.getElementById('toast-error');
    if (!toast) return;
    toast.textContent = message;
    toast.classList.remove('hidden');

    clearTimeout(toastTimeout);
    toastTimeout = setTimeout(() => {
        toast.classList.add('hidden');
    }, 4000);
}

animate();
loadPiece('honeycomb_l');
