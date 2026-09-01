
const presentationSite = document.getElementById('presentation-site');
const appContainer = document.getElementById('app');
const openViewerBtn = document.getElementById('open-viewer-btn');
const backToPresentationBtn = document.getElementById('back-to-presentation-btn');

let viewerPromise = null;

function loadViewer() {
    if (!viewerPromise) viewerPromise = import('./viewer.js');
    return viewerPromise;
}

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

window.hiveboardOpenViewer = openViewer;

if (openViewerBtn) {
    openViewerBtn.addEventListener('click', openViewer);

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

const simFrame = document.getElementById('sim-frame');
if (simFrame && 'IntersectionObserver' in window) {
    let simVisible = true;

    const tellSim = () => simFrame.contentWindow?.postMessage(
        { type: 'hiveboard-sim-visibility', visible: simVisible }, '*');

    new IntersectionObserver(([entry]) => {
        simVisible = entry.isIntersecting;
        tellSim();
    }, { rootMargin: '100px' }).observe(simFrame);

    window.addEventListener('message', (event) => {
        if (event.source === simFrame.contentWindow && event.data?.type === 'hiveboard-sim-ready') {
            tellSim();
        }
    });
}
