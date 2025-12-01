function refreshStatus() {
    fetch("/status.json")
    .then(r => r.json())
    .then(d => {
        document.getElementById("gps").innerText = d.gps;
        document.getElementById("can").innerText = d.can;
        document.getElementById("sd").innerText = d.sd;
        document.getElementById("wifi").innerText = d.wifi;
    });
}

setInterval(refreshStatus, 1500);
