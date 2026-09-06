
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

// --- Hero Tri-Video Showcase Peek Slider ---
function initHeroSlider() {
    const stage = document.getElementById('heroSliderStage');
    if (!stage) return;

    const cards = Array.from(stage.querySelectorAll('.hero-slider-card'));
    if (cards.length < 3) return;

    let currentIndex = 0;
    let isTransitioning = false;

    function applyCardClasses(newIndex, direction = 0) {
        const total = cards.length;
        const normalizedIndex = (newIndex % total + total) % total;
        const oldIndex = currentIndex;
        currentIndex = normalizedIndex;

        const centerCard = cards[currentIndex];
        const rightCard = cards[(currentIndex + 1) % total];
        const leftCard = cards[(currentIndex + 2) % total];

        cards.forEach((card) => {
            card.classList.remove('is-center', 'is-left', 'is-right');
            // Reset playback when leaving center
            if (card !== centerCard && card.classList.contains('is-playing')) {
                card.classList.remove('is-playing');
                const v = card.querySelector('.hero-video-player');
                if (v) {
                    v.pause();
                    v.controls = false;
                }
            }
        });

        centerCard.classList.add('is-center');
        rightCard.classList.add('is-right');
        leftCard.classList.add('is-left');

        // Dynamic stacking context during transition so crossing slide passes behind
        if (direction === 1) { // next (to right)
            centerCard.style.zIndex = '10';
            cards[oldIndex].style.zIndex = '7';
            leftCard.style.zIndex = '3'; // was left, crosses to right
        } else if (direction === -1) { // prev (to left)
            centerCard.style.zIndex = '10';
            cards[oldIndex].style.zIndex = '7';
            rightCard.style.zIndex = '3'; // was right, crosses to left
        } else {
            centerCard.style.zIndex = '10';
            leftCard.style.zIndex = '5';
            rightCard.style.zIndex = '5';
        }

        setTimeout(() => {
            centerCard.style.zIndex = '10';
            leftCard.style.zIndex = '5';
            rightCard.style.zIndex = '5';
            isTransitioning = false;
        }, 500);
    }

    function goToSlide(targetIndex, dir) {
        if (isTransitioning) return;
        isTransitioning = true;
        applyCardClasses(targetIndex, dir);
    }

    // Playback logic for the active center card
    function playActiveVideo(card) {
        const video = card.querySelector('.hero-video-player');
        if (!video) return;

        // Pause any other videos if playing
        cards.forEach((c) => {
            if (c !== card) {
                c.classList.remove('is-playing');
                const otherVid = c.querySelector('.hero-video-player');
                if (otherVid && !otherVid.paused) {
                    otherVid.pause();
                    otherVid.controls = false;
                }
            }
        });

        card.classList.add('is-playing');
        video.controls = true;
        video.play().catch((err) => console.log('Video playback error:', err));
    }

    // Attach click listeners to cards
    cards.forEach((card) => {
        const video = card.querySelector('.hero-video-player');
        const overlay = card.querySelector('.hero-video-overlay');

        if (video) {
            video.addEventListener('ended', () => {
                card.classList.remove('is-playing');
                video.controls = false;
            });
        }

        card.addEventListener('click', (e) => {
            if (card.classList.contains('is-left')) {
                e.preventDefault();
                goToSlide(currentIndex - 1, -1);
            } else if (card.classList.contains('is-right')) {
                e.preventDefault();
                goToSlide(currentIndex + 1, 1);
            } else if (card.classList.contains('is-center')) {
                if (!card.classList.contains('is-playing')) {
                    playActiveVideo(card);
                }
            }
        });

        if (overlay) {
            overlay.addEventListener('keydown', (e) => {
                if (card.classList.contains('is-center') && (e.key === 'Enter' || e.key === ' ')) {
                    e.preventDefault();
                    playActiveVideo(card);
                }
            });
        }
    });

    // Touch swipe navigation
    let touchStartX = 0;
    let touchStartY = 0;

    stage.addEventListener('touchstart', (e) => {
        touchStartX = e.changedTouches[0].clientX;
        touchStartY = e.changedTouches[0].clientY;
    }, { passive: true });

    stage.addEventListener('touchend', (e) => {
        const touchEndX = e.changedTouches[0].clientX;
        const touchEndY = e.changedTouches[0].clientY;
        const diffX = touchEndX - touchStartX;
        const diffY = touchEndY - touchStartY;

        if (Math.abs(diffX) > 45 && Math.abs(diffX) > Math.abs(diffY)) {
            if (diffX < 0) {
                goToSlide(currentIndex + 1, 1); // Swipe left -> next
            } else {
                goToSlide(currentIndex - 1, -1); // Swipe right -> prev
            }
        }
    }, { passive: true });

    // Initial setup
    applyCardClasses(0, 0);
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initHeroSlider);
} else {
    initHeroSlider();
}


