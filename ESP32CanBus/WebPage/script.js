document.addEventListener('DOMContentLoaded', () => {
    // Mobile nav toggle
    const toggle = document.getElementById('navToggle');
    const navCollapsed = document.getElementById('navCollapsed');
    if (toggle && navCollapsed) {
        toggle.addEventListener('click', () => {
            navCollapsed.classList.toggle('open');
            if (navCollapsed.classList.contains('open')) navCollapsed.style.display = 'flex';
            else navCollapsed.style.display = '';
        });
    }

    function refreshStatus() {
        fetch('/status.json').then(r => r.json()).then(d => {
            // Guarded updates for pages that may not have all elements
            const setIf = (id, val) => { const el = document.getElementById(id); if (el) el.innerText = (val===undefined? '--' : val); };
            setIf('gps', d.gps);
            setIf('lon', d.lon);
            setIf('lat', d.lat);
            setIf('mph', d.mph);
            setIf('sats', d.sats);
            setIf('frames', d.frames);
            setIf('can', d.can);
            setIf('sd', d.sd);
            setIf('wifi', d.wifi);
        }).catch(()=>{});
    }

    refreshStatus();
    setInterval(refreshStatus, 1500);
});
