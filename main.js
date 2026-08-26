/*
 * Entry point for the presentation site.
 *
 * The three.js viewer is a separate chunk that is only fetched when someone
 * actually opens it. #app starts hidden, so loading ~140 KB of WebGL code
 * during first paint bought nothing; this defers it until there is intent.
 */

const presentationSite = document.getElementById('presentation-site');
const appContainer = document.getElementById('app');
const openViewerBtn = document.getElementById('open-viewer-btn');
const backToPresentationBtn = document.getElementById('back-to-presentation-btn');

let viewerPromise = null;

/** Loads the viewer chunk once; repeat calls reuse the same promise. */
function loadViewer() {
    if (!viewerPromise) viewerPromise = import('./viewer.js');
    return viewerPromise;
}

/**
 * Reveals the 3D viewer, waiting for its chunk on the first open.
 * Resolves once the viewer is visible, so callers can drive it afterwards.
 */
async function openViewer() {
    if (!presentationSite || !appContainer) return;

    openViewerBtn?.classList.add('is-viewer-loading');
    try {
        await loadViewer();
    } catch (err) {
        console.error('Failed to load the 3D viewer', err);
        return;
    } finally {
        openViewerBtn?.classList.remove('is-viewer-loading');
    }

    const resetBtn = document.getElementById('reset-scene');
    if (resetBtn) resetBtn.click();

    presentationSite.style.display = 'none';
    appContainer.classList.remove('hidden-app');
    window.dispatchEvent(new Event('resize'));
}

// The "See in 3D" buttons in the page body need to await the same load.
window.hiveboardOpenViewer = openViewer;

if (openViewerBtn) {
    openViewerBtn.addEventListener('click', openViewer);

    // Warm the chunk on intent rather than on load, so visitors who never open
    // the viewer never pay for it.
    const prefetch = () => loadViewer();
    openViewerBtn.addEventListener('pointerenter', prefetch, { once: true });
    openViewerBtn.addEventListener('focusin', prefetch, { once: true });
}

if (backToPresentationBtn && presentationSite && appContainer) {
    backToPresentationBtn.addEventListener('click', () => {
        appContainer.classList.add('hidden-app');
        presentationSite.style.display = 'block';
    });
}

/*
 * The MuJoCo widget steps real physics every frame, which is wasted work while
 * it is scrolled off screen. An iframe cannot see whether it is visible in the
 * document that embeds it, so the observer has to live out here and tell it.
 */
const simFrame = document.getElementById('sim-frame');
if (simFrame && 'IntersectionObserver' in window) {
    let simVisible = true;

    const tellSim = () => simFrame.contentWindow?.postMessage(
        { type: 'hiveboard-sim-visibility', visible: simVisible }, '*');

    new IntersectionObserver(([entry]) => {
        simVisible = entry.isIntersecting;
        tellSim();
    }, { rootMargin: '100px' }).observe(simFrame);

    // The widget finishes loading long after the observer first fires, so it
    // asks for the current answer once it is ready to act on one.
    window.addEventListener('message', (event) => {
        if (event.source === simFrame.contentWindow && event.data?.type === 'hiveboard-sim-ready') {
            tellSim();
        }
    });
}
