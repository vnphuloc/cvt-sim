// DOM Elements - Canvas
const canvas = document.getElementById('cvtCanvas');
const ctx = canvas.getContext('2d');

const dynoCanvas = document.getElementById('dynoCanvas');
const dynoCtx = dynoCanvas ? dynoCanvas.getContext('2d') : null;
const dynoLiveHUD = document.getElementById('dynoLiveHUD');

// Backward-compatible rounded rectangle drawer to prevent Webview/older browser canvas crashes
function drawRoundRect(cContext, x, y, width, height, radius) {
    cContext.beginPath();
    cContext.moveTo(x + radius, y);
    cContext.lineTo(x + width - radius, y);
    cContext.quadraticCurveTo(x + width, y, x + width, y + radius);
    cContext.lineTo(x + width, y + height - radius);
    cContext.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
    cContext.lineTo(x + radius, y + height);
    cContext.quadraticCurveTo(x, y + height, x, y + height - radius);
    cContext.lineTo(x, y + radius);
    cContext.quadraticCurveTo(x, y, x + radius, y);
    cContext.closePath();
}

function drawCVT(R_front_act, R_rear_act, R_front_max, R_rear_max, rpm) {
    const maxD = Math.max(frontMaxD, rearMaxD);
    const scale = 75 / (maxD / 2); 

    const rf = R_front_act * scale;
    const rr = R_rear_act * scale;

    const rf_max = R_front_max * scale;
    const rr_max = R_rear_max * scale;

    ctx.fillStyle = '#090d16';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const cx1 = 120;
    const cy1 = 100;
    const cx2 = 380;
    const cy2 = 100;

    // Axis shaft
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(cx1, cy1);
    ctx.lineTo(cx2, cy2);
    ctx.stroke();

    // 1. Front Pulley
    ctx.beginPath();
    ctx.arc(cx1, cy1, rf_max + 10, 0, 2 * Math.PI);
    ctx.fillStyle = '#1e293b';
    ctx.fill();
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.save();
    ctx.translate(cx1, cy1);
    ctx.rotate(beltOffset * (rpm / 5000));
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1;
    for(let i=0; i<6; i++) {
        ctx.beginPath();
        ctx.moveTo(0,0);
        ctx.lineTo(Math.cos(i*Math.PI/3)*rf_max, Math.sin(i*Math.PI/3)*rf_max);
        ctx.stroke();
    }
    ctx.restore();

    // 2. Rear Pulley
    ctx.beginPath();
    ctx.arc(cx2, cy2, rr_max + 10, 0, 2 * Math.PI);
    ctx.fillStyle = '#1e293b';
    ctx.fill();
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.save();
    ctx.translate(cx2, cy2);
    ctx.rotate(-beltOffset * (rpm / 5000) * (R_front_act / R_rear_act));
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1;
    for(let i=0; i<6; i++) {
        ctx.beginPath();
        ctx.moveTo(0,0);
        ctx.lineTo(Math.cos(i*Math.PI/3)*rr_max, Math.sin(i*Math.PI/3)*rr_max);
        ctx.stroke();
    }
    ctx.restore();

    // 3. Belt with closed-path tangents
    const dist = cx2 - cx1;
    const theta = Math.asin((rr - rf) / dist);

    const xf1 = cx1 + rf * Math.sin(theta);
    const yf1 = cy1 - rf * Math.cos(theta);
    const xf2 = cx1 - rf * Math.sin(theta);
    const yf2 = cy1 + rf * Math.cos(theta);

    const xr1 = cx2 + rr * Math.sin(theta);
    const yr1 = cy2 - rr * Math.cos(theta);
    const xr2 = cx2 - rr * Math.sin(theta);
    const yr2 = cy2 + rr * Math.cos(theta);

    ctx.save();
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    ctx.beginPath();
    ctx.moveTo(xf1, yf1); 
    ctx.lineTo(xr1, yr1); 
    ctx.arc(cx2, cy2, rr, 1.5 * Math.PI + theta, 0.5 * Math.PI + theta, false);
    ctx.lineTo(xf2, yf2); 
    ctx.arc(cx1, cy1, rf, 0.5 * Math.PI + theta, 1.5 * Math.PI + theta, false);
    ctx.closePath();

    // Belt outer body
    ctx.strokeStyle = '#f43f5e';
    ctx.lineWidth = 7;
    ctx.stroke();

    // Belt moving teeth
    ctx.strokeStyle = '#090d16';
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 6]);
    ctx.lineDashOffset = beltOffset;
    ctx.stroke();
    ctx.restore();

    // Center caps
    ctx.fillStyle = '#cbd5e1';
    ctx.beginPath();
    ctx.arc(cx1, cy1, 8, 0, Math.PI*2);
    ctx.arc(cx2, cy2, 8, 0, Math.PI*2);
    ctx.fill();

    // Display live radius
    ctx.fillStyle = '#f1f5f9';
    ctx.font = 'bold 11px sans-serif';
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'center';
    ctx.fillText(`r₁ = ${R_front_act.toFixed(1)}mm`, cx1, cy1 + rf_max + 22);
    ctx.fillText(`r₂ = ${R_rear_act.toFixed(1)}mm`, cx2, cy2 + rr_max + 22);
}

function drawDynoChart(currSpeedPercent, currWheelHP, currVehicleSpeedKmh, sliderWeight, sliderSpring, frontAngle, rearAngle, maxSpeedKmh, peakRpm, speedPoints, trajectoryPoints) {
    if (!dynoCanvas || !dynoCtx) return;

    // Simulate full throttle acceleration sweep (WOT: 0% to 100% TPS) across vehicle speed using actual physics
    const dynoPoints = typeof calculateDynoCurve === 'function' 
        ? calculateDynoCurve(sliderWeight, sliderSpring, frontAngle, rearAngle, maxSpeedKmh, peakRpm)
        : [];

    if (dynoLiveHUD) {
        // Find live point index
        const safePct = Math.min(Math.max(currSpeedPercent, 0), 100);
        let liveTorqueNm = 0;
        if (dynoPoints && dynoPoints.length > 0) {
            const index = Math.min(Math.floor(safePct), dynoPoints.length - 1);
            liveTorqueNm = dynoPoints[index].torque_nm || 0;
        }
        dynoLiveHUD.innerText = `Wheel HP: ${currWheelHP.toFixed(1)}% | Wheel Torque: ${liveTorqueNm.toFixed(1)} Nm | Speed: ${currVehicleSpeedKmh.toFixed(1)} km/h (${currSpeedPercent.toFixed(1)}%)`;
    }

    // Canvas background
    dynoCtx.fillStyle = '#090d16';
    dynoCtx.fillRect(0, 0, dynoCanvas.width, dynoCanvas.height);

    const mL = 50, mR = 25, mT = 30, mB = 25;
    const pW = dynoCanvas.width - mL - mR;
    const pH = dynoCanvas.height - mT - mB;

    dynoCtx.strokeStyle = '#1e293b';
    dynoCtx.lineWidth = 1;
    dynoCtx.fillStyle = '#64748b';
    dynoCtx.font = '10px monospace';

    // Y-Axis Grid (0% to 100% Wheel HP)
    dynoCtx.textAlign = 'right';
    for (let i = 0; i <= 100; i += 25) {
        const y = mT + pH * (1 - i / 100);
        dynoCtx.beginPath();
        dynoCtx.moveTo(mL, y);
        dynoCtx.lineTo(mL + pW, y);
        dynoCtx.stroke();
        dynoCtx.fillText(`${i}% HP`, mL - 8, y + 3);
    }

    // X-Axis Grid (0% to 100% Speed / Dyno Sweep)
    dynoCtx.textAlign = 'center';
    for (let percent = 0; percent <= 100; percent += 20) {
        const x = mL + (percent / 100) * pW;
        dynoCtx.beginPath();
        dynoCtx.moveTo(x, mT);
        dynoCtx.lineTo(x, mT + pH);
        dynoCtx.stroke();
        const kmhVal = (percent / 100) * maxSpeedKmh;
        dynoCtx.fillText(`${percent}% (${kmhVal.toFixed(0)} km/h)`, x, mT + pH + 16);
    }

    // Area gradient under the dyno curve
    const gradient = dynoCtx.createLinearGradient(0, mT, 0, mT + pH);
    gradient.addColorStop(0, 'rgba(6, 182, 212, 0.28)');
    gradient.addColorStop(1, 'rgba(6, 182, 212, 0.0)');

    dynoCtx.beginPath();
    for (let i = 0; i < dynoPoints.length; i++) {
        const xPos = mL + (dynoPoints[i].x / 100) * pW;
        const yPos = mT + pH * (1 - dynoPoints[i].y / 100);
        if (i === 0) dynoCtx.moveTo(xPos, yPos);
        else dynoCtx.lineTo(xPos, yPos);
    }
    dynoCtx.lineTo(mL + pW, mT + pH);
    dynoCtx.lineTo(mL, mT + pH);
    dynoCtx.closePath();
    dynoCtx.fillStyle = gradient;
    dynoCtx.fill();

    // Wheel Torque curve line (Emerald Green)
    if (dynoPoints.length > 0 && dynoPoints[0].torque_scaled !== undefined) {
        dynoCtx.beginPath();
        dynoCtx.lineWidth = 2.8;
        dynoCtx.strokeStyle = '#10b981'; // Emerald
        for (let i = 0; i < dynoPoints.length; i++) {
            const xPos = mL + (dynoPoints[i].x / 100) * pW;
            const yPos = mT + pH * (1 - Math.min(100, dynoPoints[i].torque_scaled) / 100);
            if (i === 0) dynoCtx.moveTo(xPos, yPos);
            else dynoCtx.lineTo(xPos, yPos);
        }
        dynoCtx.stroke();
    }

    // Dyno power curve line
    dynoCtx.beginPath();
    dynoCtx.lineWidth = 2.8;
    dynoCtx.strokeStyle = '#06b6d4';
    for (let i = 0; i < dynoPoints.length; i++) {
        const xPos = mL + (dynoPoints[i].x / 100) * pW;
        const yPos = mT + pH * (1 - dynoPoints[i].y / 100);
        if (i === 0) dynoCtx.moveTo(xPos, yPos);
        else dynoCtx.lineTo(xPos, yPos);
    }
    dynoCtx.stroke();

    // Draw Potential Power Curve (speedPoints)
    if (speedPoints && speedPoints.length > 0) {
        dynoCtx.beginPath();
        dynoCtx.lineWidth = 2.0;
        dynoCtx.strokeStyle = 'rgba(244, 63, 94, 0.8)'; // Rose color, slightly transparent to not overpower
        for (let i = 0; i < speedPoints.length; i++) {
            const xPos = mL + (speedPoints[i].x / 100) * pW;
            const yPos = mT + pH * (1 - speedPoints[i].wheelHP / 100);
            if (i === 0) dynoCtx.moveTo(xPos, yPos);
            else dynoCtx.lineTo(xPos, yPos);
        }
        dynoCtx.stroke();
    }

    // Draw Actual Sweep Trajectory (trajectoryPoints)
    if (trajectoryPoints && trajectoryPoints.length > 0) {
        dynoCtx.save();
        dynoCtx.beginPath();
        dynoCtx.lineWidth = 2.2;
        dynoCtx.strokeStyle = 'rgba(245, 158, 11, 0.8)'; // Amber color
        dynoCtx.setLineDash([4, 4]);
        for (let i = 0; i < trajectoryPoints.length; i++) {
            const xPos = mL + (trajectoryPoints[i].x / 100) * pW;
            const yPos = mT + pH * (1 - trajectoryPoints[i].y / 100);
            if (i === 0) dynoCtx.moveTo(xPos, yPos);
            else dynoCtx.lineTo(xPos, yPos);
        }
        dynoCtx.stroke();
        dynoCtx.restore();
    }

    // Draw current vehicle operating point on dyno curve
    const safePct = Math.min(Math.max(currSpeedPercent, 0), 100);
    const liveX = mL + (safePct / 100) * pW;
    const liveY = mT + pH * (1 - currWheelHP / 100);

    // HP Point
    dynoCtx.save();
    dynoCtx.strokeStyle = '#f43f5e';
    dynoCtx.lineWidth = 1.2;
    dynoCtx.setLineDash([4, 4]);

    // Vertical crosshair
    dynoCtx.beginPath();
    dynoCtx.moveTo(liveX, mT + pH);
    dynoCtx.lineTo(liveX, liveY);
    dynoCtx.stroke();

    // Horizontal crosshair
    dynoCtx.beginPath();
    dynoCtx.moveTo(mL, liveY);
    dynoCtx.lineTo(liveX, liveY);
    dynoCtx.stroke();
    dynoCtx.restore();

    // Glowing indicator dot (HP)
    dynoCtx.beginPath();
    dynoCtx.arc(liveX, liveY, 6 + Math.abs(Math.sin(Date.now() / 150) * 3), 0, 2 * Math.PI);
    dynoCtx.fillStyle = 'rgba(244, 63, 94, 0.35)';
    dynoCtx.fill();

    dynoCtx.beginPath();
    dynoCtx.arc(liveX, liveY, 4, 0, 2 * Math.PI);
    dynoCtx.fillStyle = '#ffffff';
    dynoCtx.fill();
    dynoCtx.strokeStyle = '#f43f5e';
    dynoCtx.lineWidth = 2;
    dynoCtx.stroke();

    // Glowing indicator dot (Torque)
    if (dynoPoints && dynoPoints.length > 0 && dynoPoints[0].torque_scaled !== undefined) {
        const index = Math.min(Math.floor(safePct), dynoPoints.length - 1);
        const liveTorqueY = mT + pH * (1 - Math.min(100, dynoPoints[index].torque_scaled) / 100);
        
        dynoCtx.beginPath();
        dynoCtx.arc(liveX, liveTorqueY, 6 + Math.abs(Math.sin(Date.now() / 150) * 3), 0, 2 * Math.PI);
        dynoCtx.fillStyle = 'rgba(16, 185, 129, 0.35)';
        dynoCtx.fill();

        dynoCtx.beginPath();
        dynoCtx.arc(liveX, liveTorqueY, 4, 0, 2 * Math.PI);
        dynoCtx.fillStyle = '#ffffff';
        dynoCtx.fill();
        dynoCtx.strokeStyle = '#10b981';
        dynoCtx.lineWidth = 2;
        dynoCtx.stroke();
    }
}

// Animation Loop
function animate() {
    const rpm = parseFloat(rpmSlider.value);
    beltOffset -= (rpm / 60) * 0.15;
    
    // Smoothen transitions
    shiftFactor += (targetShiftFactor - shiftFactor) * 0.08;
    if (Math.abs(shiftFactor - targetShiftFactor) < 0.001) {
        shiftFactor = targetShiftFactor;
    }

    updatePhysicsAndVisuals();
    animationFrameId = requestAnimationFrame(animate);
}

// Event Listeners setup
function setupEventListeners() {
    rpmSlider.addEventListener('input', updatePhysicsAndVisuals);
    weightSlider.addEventListener('input', updatePhysicsAndVisuals);
    springSlider.addEventListener('input', updatePhysicsAndVisuals);
    frontAngleSlider.addEventListener('input', updatePhysicsAndVisuals);
    rearAngleSlider.addEventListener('input', updatePhysicsAndVisuals);
    peakRpmSlider.addEventListener('input', updatePhysicsAndVisuals);
}

// Start Simulator
window.onload = function() {
    setupEventListeners();
    animate();
};