document.addEventListener('DOMContentLoaded', () => {
    // Mobile nav removed per UI preference; nav remains visible.

    // Safe setter
    const setIf = (id, val) => { const el = document.getElementById(id); if (el) el.innerText = (val===undefined? '--' : val); };

    function refreshStatus() {
        fetch('/status.json').then(r => r.json()).then(d => {
            setIf('gps', d.gps);
            setIf('lon', d.lon);
            setIf('lat', d.lat);
            setIf('mph', d.mph);
            setIf('sats', d.sats);
            setIf('frames', d.frames);
            setIf('can', d.can);
            setIf('sd', d.sd);
            setIf('wifi', d.wifi);
            // update gauge from mph if available
            updateGauge(Number(d.mph) || 0);
        }).catch(()=>{});
    }

    // Gauge needle control (simple rotation based on mph)
    function updateGauge(mph){
        const needle = document.getElementById('gaugeNeedle');
        const big = document.getElementById('mph');
        const max = 160; // gauge max mph
        const clamped = Math.max(0, Math.min(max, mph));
        const angle = (clamped / max) * 160 - 80; // -80..+80 degrees
        if(needle) needle.style.transform = `rotate(${angle}deg)`;
        if(big) big.innerText = (isNaN(mph) ? '--' : String(Math.round(mph)));
    }

    // Frames rendering
    function hashString(s){
        let h=0; for(let i=0;i<s.length;i++){ h = ((h<<5)-h) + s.charCodeAt(i); h |= 0; } return Math.abs(h);
    }
    function escapeHtml(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }

    function renderFrames(frames){
        const grid = document.getElementById('framesGrid'); if(!grid) return;
        // accept array or object.holder
        const arr = Array.isArray(frames) ? frames : (frames && frames.records) ? frames.records : [];
        // keep most recent first
        grid.innerHTML = '';
        const max = 60;
        arr.slice(0, max).forEach(f => {
            const id = f.id || f.canId || f.id_str || (f[0] || '0x000');
            const data = f.data || f.payload || f.d || f.data_hex || f[1] || '';
            const ts = f.ts || f.time || f.t || new Date().toLocaleTimeString();
            const el = document.createElement('div'); el.className = 'frame-row';
            const h = hashString(String(id)) % 360;
            el.style.borderLeft = `6px solid hsl(${h} 90% 55%)`;
            el.innerHTML = `<div style="display:flex;justify-content:space-between;align-items:center"><div class='frame-id'>${escapeHtml(String(id))}</div><div class='muted'>${escapeHtml(String(ts))}</div></div><div style="margin-top:8px;font-family:monospace;font-size:13px">${escapeHtml(String(data))}</div>`;
            grid.appendChild(el);
        });
    }

    function fetchFrames(){
        // try JSON endpoint first
        fetch('/frames').then(r=>r.json()).then(d => { renderFrames(d); }).catch(()=>{
            // fallback to text lines: id:data:timestamp
            fetch('/frames.txt').then(r=>r.text()).then(txt => {
                const lines = txt.trim().split('\n').filter(Boolean).map(l => {
                    const parts = l.split('|');
                    return { id: parts[0], data: parts[1], ts: parts[2] };
                });
                renderFrames(lines);
            }).catch(()=>{});
        });
    }

    // initial load + intervals
    refreshStatus(); fetchFrames();
    setInterval(refreshStatus, 1500);
    setInterval(fetchFrames, 700);
});
