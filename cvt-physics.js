// --- PHYSICAL & MECHANICAL CONSTANTS ---
let CONST = {
    GEAR_RATIO_MAX: 2.64,
    GEAR_RATIO_MIN: 0.82,
    FINAL_REDUCTION: 10.5,
    WHEEL_DIAMETER_M: 0.58,
    RAMP_ANGLE_DEG: 20.0,
    STD_FRONT_ANGLE: 15.0,
    STD_REAR_ANGLE: 16.0,
    STD_ROLLER_WEIGHT_G: 14.0,
    BASE_ROLLER_RADIUS_M: 0.030,
    BASE_CVT_EFFICIENCY: 0.82,
    MAX_ENGINE_RPM: 10000,
    CLUTCH_ENGAGE_RPM: 3000,
    REAL_DYNO_PEAK_RPM: 7200,
    ENGINE_BANDWIDTH: 1500
};

// DOM Elements - Inputs
const rpmSlider = document.getElementById('rpmSlider');
const weightSlider = document.getElementById('weightSlider');
const springSlider = document.getElementById('springSlider');
const frontAngleSlider = document.getElementById('frontAngleSlider');
const rearAngleSlider = document.getElementById('rearAngleSlider');
const peakRpmSlider = document.getElementById('peakRpmSlider');

const rpmVal = document.getElementById('rpmVal');
const weightVal = document.getElementById('weightVal');
const springVal = document.getElementById('springVal');
const frontAngleVal = document.getElementById('frontAngleVal');
const rearAngleVal = document.getElementById('rearAngleVal');
const peakRpmVal = document.getElementById('peakRpmVal');

const frontMaxDInput = document.getElementById('frontMaxDInput');
const rearMaxDInput = document.getElementById('rearMaxDInput');

// DOM Elements - Outputs/HUD
const statusBanner = document.getElementById('statusBanner');
const debugConsole = document.getElementById('debugConsole');
const ratioDisplay = document.getElementById('ratioDisplay');
const shiftDisplay = document.getElementById('shiftDisplay');

function initConstantInputs() {
    document.getElementById('const_gear_max').value = CONST.GEAR_RATIO_MAX;
    document.getElementById('const_gear_min').value = CONST.GEAR_RATIO_MIN;
    document.getElementById('const_final_red').value = CONST.FINAL_REDUCTION;
    document.getElementById('const_wheel_d').value = CONST.WHEEL_DIAMETER_M;
    document.getElementById('const_peak_rpm').value = CONST.REAL_DYNO_PEAK_RPM;
    document.getElementById('const_bandwidth').value = CONST.ENGINE_BANDWIDTH;
    peakRpmSlider.value = CONST.REAL_DYNO_PEAK_RPM;
}

window.addEventListener('DOMContentLoaded', initConstantInputs);

function applySystemConstants() {
    const values = {
        gearMax: parseFloat(document.getElementById('const_gear_max').value),
        gearMin: parseFloat(document.getElementById('const_gear_min').value),
        finalReduction: parseFloat(document.getElementById('const_final_red').value),
        wheelDiameter: parseFloat(document.getElementById('const_wheel_d').value),
        peakRpm: parseFloat(document.getElementById('const_peak_rpm').value),
        bandwidth: parseFloat(document.getElementById('const_bandwidth').value)
    };

    if (!Number.isFinite(values.gearMax) || !Number.isFinite(values.gearMin) ||
        values.gearMin <= 0 || values.gearMax <= values.gearMin ||
        !Number.isFinite(values.finalReduction) || values.finalReduction <= 0 ||
        !Number.isFinite(values.wheelDiameter) || values.wheelDiameter <= 0 ||
        !Number.isFinite(values.peakRpm) || values.peakRpm < CONST.CLUTCH_ENGAGE_RPM ||
        values.peakRpm > CONST.MAX_ENGINE_RPM ||
        !Number.isFinite(values.bandwidth) || values.bandwidth <= 0) {
        alert("Invalid system constants. Check ratio order, positive dimensions, and RPM ranges.");
        return;
    }

    CONST.GEAR_RATIO_MAX = values.gearMax;
    CONST.GEAR_RATIO_MIN = values.gearMin;
    CONST.FINAL_REDUCTION = values.finalReduction;
    CONST.WHEEL_DIAMETER_M = values.wheelDiameter;
    CONST.REAL_DYNO_PEAK_RPM = values.peakRpm;
    CONST.ENGINE_BANDWIDTH = values.bandwidth;
    peakRpmSlider.value = values.peakRpm;
    updatePhysicsAndVisuals();
}

// Dynamic State Variables (Global for shared access with cvt-canvas.js)
let frontMaxD = 130; // mm
let rearMaxD = 162;  // mm
let animationFrameId;
let beltOffset = 0;

// Easing interpolation values for physical realism
let shiftFactor = 0.0;       // Current animated shift state (0.0 to 1.0)
let targetShiftFactor = 0.0; // Target equilibrium shift state based on physics calculations

// Apply new custom pulley sizes
function applyPulleySizes() {
    const fIn = parseFloat(frontMaxDInput.value);
    const rIn = parseFloat(rearMaxDInput.value);

    if (isNaN(fIn) || fIn < 100 || fIn > 250 || isNaN(rIn) || rIn < 100 || rIn > 250) {
        alert("⚠️ Error: Pulley diameter must be within the safe range (100mm - 250mm).");
        return;
    }

    frontMaxD = fIn;
    rearMaxD = rIn;
    updatePhysicsAndVisuals();
}

function calculateOptimalTuning(peakRpm, weight, springKg, frontAngleDeg, rearAngleDeg) {
    const omega_peak = (peakRpm * 2 * Math.PI) / 60;
    const theta_ramp_rad = (CONST.RAMP_ANGLE_DEG * Math.PI) / 180;
    const rearAngleRad = (rearAngleDeg * Math.PI) / 180;
    const frontAngleRad = (frontAngleDeg * Math.PI) / 180;
    
    // Mid-shift (sf = 0.5) baseline equilibrium target
    const activeSpring_mid = springKg * (0.8 + 0.4 * 0.5); // 1.0 * springKg
    const F_rear_axial_spring = activeSpring_mid * 9.81;
    const F_rear_belt_tension = F_rear_axial_spring / Math.tan(rearAngleRad);
    
    // Target forceRatio for sf = 0.5 is 1.25 (since targetSF = (Ratio - 0.5)/1.5 => 0.5 = (Ratio - 0.5)/1.5 => Ratio = 1.25)
    const targetForceRatio = 1.25;
    const F_front_axial_target = targetForceRatio * F_rear_belt_tension;
    const F_c_to_axial_target = F_front_axial_target * Math.tan(frontAngleRad);
    const Fc_target = F_c_to_axial_target / Math.tan(theta_ramp_rad);
    
    // 1. OPTIMAL ROLLER WEIGHT (given current spring Force and front angle)
    const r_bi_mid = 0.045; // meters at sf = 0.5
    const M_tot_opt = Fc_target / (Math.pow(omega_peak, 2) * r_bi_mid);
    const optWeight_g = Math.min(Math.max((M_tot_opt / 6) * 1000, 3.0), 25.0);

    // 2. OPTIMAL SPRING FORCE (given current roller weight and front angle)
    const m_curr = weight / 1000;
    const M_tot_curr = 6 * m_curr;
    const Fc_curr_at_peak = M_tot_curr * Math.pow(omega_peak, 2) * r_bi_mid;
    const F_front_axial_curr_at_peak = (Fc_curr_at_peak * Math.tan(theta_ramp_rad)) / Math.tan(frontAngleRad);
    const F_rear_belt_tension_opt = F_front_axial_curr_at_peak / targetForceRatio;
    const F_rear_axial_spring_opt = F_rear_belt_tension_opt * Math.tan(rearAngleRad);
    const optSpring_kg = Math.min(Math.max(F_rear_axial_spring_opt / 9.81, 15.0), 65.0);

    // 3. OPTIMAL FRONT PULLEY ANGLE (given current roller weight and spring force)
    const tan_theta1_opt = (Fc_curr_at_peak * Math.tan(theta_ramp_rad)) / (targetForceRatio * F_rear_belt_tension);
    const theta1_opt_deg = Math.min(Math.max(Math.atan(tan_theta1_opt) * (180 / Math.PI), 10.0), 20.0);

    return {
        optWeight: optWeight_g,
        optSpring: optSpring_kg,
        optFrontAngle: theta1_opt_deg
    };
}

function calculateEnginePowerPercent(rpm, peakRpm) {
    return 100 * Math.exp(-0.5 * Math.pow((rpm - peakRpm) / CONST.ENGINE_BANDWIDTH, 2));
}

function calculateEngineTorqueNm(rpm, peakRpm) {
    const safeRpm = Math.max(rpm, 1);
    const powerFraction = calculateEnginePowerPercent(safeRpm, peakRpm) / 100;
    return 14.8 * powerFraction * (peakRpm / safeRpm);
}

function calculateDynoCurve(sliderWeight, sliderSpring, frontAngle, rearAngle, maxSpeedKmh, peakRpm) {
    const dynoPoints = [];
    const frontAngleRad = (frontAngle * Math.PI) / 180;
    const rearAngleRad = (rearAngle * Math.PI) / 180;

    // 1. Vòng tua hãm tự nhiên của bộ CVT (Shift RPM) khi thốc 100% TPS
    const springFactor = sliderSpring / 40.0;
    const weightFactor = Math.max(0.1, (sliderWeight * 6) / 114.0);
    const angleFactor = Math.tan(15.0 * Math.PI / 180) / Math.tan(frontAngleRad);
    
    // Bi càng nhẹ hoặc lò xo càng cứng -> Shift RPM càng cao
    let shiftRPM = 8000 * Math.sqrt(springFactor / weightFactor) * Math.sqrt(angleFactor);
    shiftRPM = Math.max(CONST.CLUTCH_ENGAGE_RPM, Math.min(shiftRPM, CONST.MAX_ENGINE_RPM));

    // 2. Khả năng biến thiên & Lực kẹp puly
    // Bi quá nhẹ (< 5g) thì không đủ lực thắng lò xo để bung hết puly
    const maxShiftFactor = Math.min(1.0, Math.max(0.0, (sliderWeight - 2.0) / 7.0)); // 2g trở xuống kẹt 0%
    const R_min_actual = CONST.GEAR_RATIO_MAX - (CONST.GEAR_RATIO_MAX - CONST.GEAR_RATIO_MIN) * maxShiftFactor;

    // 3. Điểm hết puly (Shift-out speed %)
    const wheelCirc = CONST.WHEEL_DIAMETER_M * Math.PI;
    const speedAtShiftOutKmh = (shiftRPM / (R_min_actual * CONST.FINAL_REDUCTION)) * wheelCirc * 60 / 1000;
    const shiftOutPct = Math.min(90, Math.max(30, (speedAtShiftOutKmh / maxSpeedKmh) * 100));
    const clutchEngagePct = 15;

    // 4. Công suất cơ sở của động cơ phát ra tại Shift RPM
    for (let pct = 0; pct <= 100; pct += 1) {
        const v_kmh = (pct / 100) * maxSpeedKmh;
        let dynoRPM = shiftRPM;
        let clutchFactor = 1.0;
        let overdriveLoss = 1.0;

        // Vòng tua bắt buộc nếu xe đạt vận tốc v_kmh tại tỷ số truyền hiện tại
        const reqRPM = (v_kmh * 1000 / 60) / wheelCirc * (R_min_actual * CONST.FINAL_REDUCTION);

        if (pct < clutchEngagePct) {
            // Giai đoạn 1: Bắt bố 3 càng (0% - 15%)
            const p = pct / clutchEngagePct;
            dynoRPM = CONST.CLUTCH_ENGAGE_RPM + p * (shiftRPM - CONST.CLUTCH_ENGAGE_RPM);
            clutchFactor = Math.sin((p * Math.PI) / 2);
        } else if (pct <= shiftOutPct) {
            // Giai đoạn 2: Biến thiên CVT giữ tua phẳng (15% - shiftOutPct)
            const sp = (pct - clutchEngagePct) / (shiftOutPct - clutchEngagePct);
            dynoRPM = shiftRPM + sp * 250;
        } else {
            // Giai đoạn 3: Overdrive hết puly (Tua máy bị ép leo dốc)
            const op = (pct - shiftOutPct) / (100 - shiftOutPct || 1);
            dynoRPM = (shiftRPM + 250) + op * 2000;
            // Công suất vuốt dốc êm xuống ở dải cuối
            overdriveLoss = 1.0 - (op * 0.35);
            if (shiftRPM < peakRpm - 600) {
                overdriveLoss -= op * 0.3; // Xe đè tua thì đuôi tụt dốc nhanh hơn
            }
            overdriveLoss = Math.max(0.1, overdriveLoss);
        }

        // Thay vì cắt phựt về 0 (powerCut = 0.0), ta vuốt dốc mượt mà khi chạm trần RPM
        let revLimitFactor = 1.0;
        if (reqRPM > CONST.MAX_ENGINE_RPM) {
            const overRPM = reqRPM - CONST.MAX_ENGINE_RPM;
            // Vuốt dốc mềm mại xuống chứ không rơi thẳng đứng
            revLimitFactor = Math.max(0.0, 1.0 - (overRPM / 1500));
            dynoRPM = CONST.MAX_ENGINE_RPM;
        }

        // Tỷ số truyền CVT thực tế tại tốc độ v
        let R_cvt;
        if (pct < clutchEngagePct) {
            R_cvt = CONST.GEAR_RATIO_MAX;
        } else if (pct <= shiftOutPct) {
            const sp = (pct - clutchEngagePct) / (shiftOutPct - clutchEngagePct);
            R_cvt = CONST.GEAR_RATIO_MAX - (CONST.GEAR_RATIO_MAX - R_min_actual) * sp;
        } else {
            R_cvt = R_min_actual;
        }

        const enginePower = calculateEnginePowerPercent(dynoRPM, peakRpm);

        // Hiện tượng võng lõm do trượt dây curoa khi bi nhẹ (< 10g)
        let slipDip = 1.0;
        if (sliderWeight < 10.0 && pct > 15 && pct < 60) {
            const dipProgress = (pct - 15) / 45;
            const dipDepth = (10.0 - sliderWeight) * 0.025; // Giảm độ võng vừa phải
            slipDip = 1.0 - Math.sin(dipProgress * Math.PI) * dipDepth;
        }

        // Hiệu suất truyền động cơ bản
        let eff = CONST.BASE_CVT_EFFICIENCY; // 0.82
        const angleMismatch = (Math.abs(frontAngle - 15.0) + Math.abs(rearAngle - 16.0)) * 0.015;
        eff = Math.max(0.5, eff - angleMismatch);

        // Công suất bánh xe Wheel HP (%) - vuốt dốc mềm ở cuối
        const P_wheel = enginePower * eff * clutchFactor * overdriveLoss * revLimitFactor * slipDip;

        // Mô-men xoắn bánh xe Wheel Torque (Nm)
        const totalRatio = R_cvt * CONST.FINAL_REDUCTION;
        const T_engine = calculateEngineTorqueNm(dynoRPM, peakRpm);
        const T_wheel = T_engine * totalRatio * eff * clutchFactor * overdriveLoss * revLimitFactor * slipDip;

        // Scale đường Torque: map mượt mà theo chiều cao canvas
        const T_wheel_scaled = (T_wheel / 250) * 100;

        dynoPoints.push({
            x: pct,
            y: Math.max(0, P_wheel),
            torque_nm: T_wheel,
            torque_scaled: Math.max(0, T_wheel_scaled),
            kmh: v_kmh,
            rpm: dynoRPM
        });
    }
    return dynoPoints;
}

function calculateCurvePoints(currSpeedPercent, currWheelHP, sliderRPM, sliderWeight, sliderSpring, frontAngle, rearAngle, maxSpeedKmh, peakRpm) {
    // 1. Calculate Potential Line Points (Solid Rose Red)
    const maxReachableSpeedPercentAtSliderRPM = (sliderRPM / CONST.MAX_ENGINE_RPM) * 100;

    const speedPoints = [];
    for (let pct = 0; pct <= 100; pct += 1) {
        const v = (pct / 100) * maxSpeedKmh;
        const wheelRPM = v / (CONST.WHEEL_DIAMETER_M * Math.PI * 60 / 1000);
        
        const requiredRatio = sliderRPM / (wheelRPM * CONST.FINAL_REDUCTION || 0.001);
        const actualRatio = Math.min(Math.max(requiredRatio, CONST.GEAR_RATIO_MIN), CONST.GEAR_RATIO_MAX);
        const calculatedRPM = wheelRPM * CONST.FINAL_REDUCTION * actualRatio;
        
        const sf_potential = Math.min(Math.max((CONST.GEAR_RATIO_MAX - actualRatio) / (CONST.GEAR_RATIO_MAX - CONST.GEAR_RATIO_MIN || 1), 0), 1);

        let engineHPAtV = calculateEnginePowerPercent(calculatedRPM, peakRpm);
        let effAtV = CONST.BASE_CVT_EFFICIENCY;
        
        // 1. Friction/Heat loss from angle mismatch (belt deformation)
        const angleDeformationLoss = (Math.abs(frontAngle - 15.0) + Math.abs(rearAngle - 15.0)) * 0.015;
        effAtV -= angleDeformationLoss;

        if (pct < 12) effAtV *= (pct / 12);
        if (calculatedRPM > 8500) effAtV -= ((calculatedRPM - 8500) / 1500) * 0.10;
        effAtV = Math.max(0, Math.min(effAtV, 0.85));

        const m_pot = sliderWeight / 1000;
        const M_pot_tot = 6 * m_pot;
        const omega_pot = (calculatedRPM * 2 * Math.PI) / 60;
        const r_bi_pot = CONST.BASE_ROLLER_RADIUS_M + CONST.BASE_ROLLER_RADIUS_M * sf_potential;
        const Fc_pot = M_pot_tot > 0 ? M_pot_tot * Math.pow(omega_pot, 2) * r_bi_pot : 0;

        const theta_ramp_rad = (CONST.RAMP_ANGLE_DEG * Math.PI) / 180;
        const frontAngleRad = (frontAngle * Math.PI) / 180;
        const F_front_ax_pot = (Fc_pot * Math.tan(theta_ramp_rad)) / Math.tan(frontAngleRad);

        const activeSpring_pot = sliderSpring * (0.8 + 0.4 * sf_potential);
        const F_rear_ax_pot = activeSpring_pot * 9.81;
        const rearAngleRad = (rearAngle * Math.PI) / 180;
        const F_rear_belt_pot = F_rear_ax_pot / Math.tan(rearAngleRad);

        const potForceRatio = F_front_ax_pot / (F_rear_belt_pot || 1);

        // --- BELT SLIP SIMULATION ---
        let slipSeverityPenalty = 1.0;

        // A. Rear Pulley Slip (High Speed / Overdrive)
        const refRearClamp = 45 / Math.tan(15 * Math.PI / 180);
        const actualRearClamp = sliderSpring / Math.tan(rearAngle * Math.PI / 180);
        const rearClampFactor = actualRearClamp / refRearClamp;

        if (rearClampFactor < 0.90) {
            const highSpeedFactor = Math.max(0, (pct - 40) / 60); 
            const severity = (0.90 - rearClampFactor) * 2.0; 
            slipSeverityPenalty -= severity * highSpeedFactor;
        }

        // B. Front Pulley Slip (Launch / Acceleration)
        if (sliderWeight < 12.0) {
            const lowSpeedFactor = Math.max(0, (40 - pct) / 40); 
            const severity = (12.0 - sliderWeight) * 0.03; 
            slipSeverityPenalty -= severity * lowSpeedFactor;
        }

        // C. Dynamic Equilibrium Clamping Penalty (Potential Curve specific)
        let clampingPenalty = 1.0;
        if (potForceRatio < 0.65) {
            clampingPenalty = Math.max(0.35, potForceRatio / 0.65);
        }
        
        slipSeverityPenalty = Math.max(0.20, slipSeverityPenalty);

        let wheelHPAtV = engineHPAtV * effAtV * clampingPenalty * slipSeverityPenalty;

        if (pct > maxReachableSpeedPercentAtSliderRPM) {
            const excessPct = pct - maxReachableSpeedPercentAtSliderRPM;
            const smoothDrop = Math.max(0, 1 - (excessPct / 25.0));
            wheelHPAtV = wheelHPAtV * Math.pow(smoothDrop, 1.5);
        }
        
        speedPoints.push({ x: pct, wheelHP: wheelHPAtV });
    }

    // 2. Calculate Actual Operating Sweep Trajectory Line Points (Dashed Amber)
    const trajectoryPoints = [];
    for (let tempRpm = 1500; tempRpm <= CONST.MAX_ENGINE_RPM; tempRpm += 100) {
        const tempM = sliderWeight / 1000;
        const tempM_total = 6 * tempM;
        const tempOmega = (tempRpm * 2 * Math.PI) / 60;
        
        let tempSF = 0.5; 
        for (let iter = 0; iter < 3; iter++) {
            const r_bi_temp = CONST.BASE_ROLLER_RADIUS_M + CONST.BASE_ROLLER_RADIUS_M * tempSF;
            const F_c_temp = tempM_total > 0 ? tempM_total * Math.pow(tempOmega, 2) * r_bi_temp : 0;
            const F_c_axial_temp = F_c_temp * Math.tan((CONST.RAMP_ANGLE_DEG * Math.PI) / 180);
            const F_front_axial_temp = F_c_axial_temp / Math.tan((frontAngle * Math.PI) / 180);
            
            const activeSpringKg_temp = sliderSpring * (0.8 + 0.4 * tempSF);
            const F_rear_axial_spring_temp = activeSpringKg_temp * 9.81;
            const F_rear_belt_tension_temp = F_rear_axial_spring_temp / Math.tan((rearAngle * Math.PI) / 180);
            
            const forceRatio_temp = F_front_axial_temp / (F_rear_belt_tension_temp || 1);
            tempSF = Math.min(Math.max((forceRatio_temp - 0.5) / 1.5, 0), 1);
        }
        
        const tempCvtRatio = CONST.GEAR_RATIO_MAX - (CONST.GEAR_RATIO_MAX - CONST.GEAR_RATIO_MIN) * tempSF;
        const tempWheelRPM = tempRpm / (tempCvtRatio * CONST.FINAL_REDUCTION);
        const tempVehicleSpeedKmh = tempWheelRPM * CONST.WHEEL_DIAMETER_M * Math.PI * 60 / 1000;
        const tempSpeedPercent = (tempVehicleSpeedKmh / maxSpeedKmh) * 100;
        
        const tempEngineHP = calculateEnginePowerPercent(tempRpm, peakRpm);
        let tempEff = CONST.BASE_CVT_EFFICIENCY;
        if (tempSpeedPercent < 12) tempEff *= (tempSpeedPercent / 12);
        if (tempRpm > 8500) tempEff -= ((tempRpm - 8500) / 1500) * 0.10;
        tempEff = Math.max(0, Math.min(tempEff, 0.85));
        
        trajectoryPoints.push({ x: Math.min(100, tempSpeedPercent), y: tempEngineHP * tempEff });
    }

    return { speedPoints, trajectoryPoints };
}

function updateStatusText(rpm, shiftFactor, peakRpm) {
    if (rpm <= 1800) {
        statusBanner.innerText = "Status: Idle";
        statusBanner.className = "px-4 py-2 rounded-lg font-bold text-center text-sm shadow-md bg-slate-800 text-slate-300 w-full md:w-auto";
    } else if (rpm >= peakRpm - 200 && rpm <= peakRpm + 300 && shiftFactor > 0.75) {
        statusBanner.innerText = "Status: Peak Torque & Power Zone";
        statusBanner.className = "px-4 py-2 rounded-lg font-bold text-center text-sm shadow-md bg-emerald-500/20 text-emerald-400 border border-emerald-500/30 w-full md:w-auto";
    } else if (rpm < peakRpm - 200 && shiftFactor > 0.65) {
        statusBanner.innerText = "Status: Sluggish Acceleration (Premature Upshift / Bogging)";
        statusBanner.className = "px-4 py-2 rounded-lg font-bold text-center text-sm shadow-md bg-amber-500/20 text-amber-400 border border-amber-500/30 w-full md:w-auto";
    } else if (rpm > peakRpm - 1000 && shiftFactor < 0.75) {
        statusBanner.innerText = "Status: Over-revving (Rollers too light or Spring too stiff)";
        statusBanner.className = "px-4 py-2 rounded-lg font-bold text-center text-sm shadow-md bg-rose-500/20 text-rose-400 border border-rose-500/30 w-full md:w-auto";
    } else {
        statusBanner.innerText = "Status: Acceleration Zone";
        statusBanner.className = "px-4 py-2 rounded-lg font-bold text-center text-sm shadow-md bg-sky-500/20 text-sky-400 border border-sky-500/30 w-full md:w-auto";
    }
}

function updateDebugLog(p) {
    const maxReachableSpeedPercentAtSliderRPM = (p.rpm / CONST.MAX_ENGINE_RPM) * 100;
    const maxReachableKmhAtSliderRPM = (p.rpm / (CONST.GEAR_RATIO_MIN * CONST.FINAL_REDUCTION)) * CONST.WHEEL_DIAMETER_M * Math.PI * 60 / 1000;
    const opt = p.optTuning;

    debugConsole.innerHTML = `
<div class="border-b border-cyan-950 pb-2 mb-2 font-bold text-cyan-400">== SYSTEM CONSTANTS ==</div>
<div class="pl-2 grid grid-cols-2 gap-x-2 text-[10px] text-cyan-200/80 mb-3">
<div>CVT Ratio Max (Low Gear): ${CONST.GEAR_RATIO_MAX}</div>
<div>CVT Ratio Min (Overdrive): ${CONST.GEAR_RATIO_MIN}</div>
<div>Final Reduction: ${CONST.FINAL_REDUCTION}</div>
<div>Wheel Diameter: ${CONST.WHEEL_DIAMETER_M} m</div>
<div>Base CVT Efficiency: ${CONST.BASE_CVT_EFFICIENCY}</div>
<div>Ramp Angle (Front): ${CONST.RAMP_ANGLE_DEG}°</div>
<div>Std Front Angle: ${CONST.STD_FRONT_ANGLE}°</div>
<div>Std Rear Angle: ${CONST.STD_REAR_ANGLE}°</div>
<div>Base Roller Radius: ${CONST.BASE_ROLLER_RADIUS_M} m</div>
<div>Max Engine RPM: ${CONST.MAX_ENGINE_RPM}</div>
<div>Dyno Peak RPM: ${CONST.REAL_DYNO_PEAK_RPM}</div>
<div>Engine Bandwidth: ${CONST.ENGINE_BANDWIDTH}</div>
</div>

<div class="border-b border-emerald-950 pb-2 mb-2 font-bold text-slate-300">== INPUT PARAMETERS & KINEMATICS ==</div>
<div>[1] Crankshaft Angular Velocity (&omega;):</div>
<div class="pl-4 text-emerald-400/80">&omega; = (2 * &pi; * RPM) / 60</div>
<div class="pl-4 text-slate-300">&omega; = (2 * 3.1416 * ${p.rpm}) / 60 = <span class="text-rose-400 font-semibold">${p.omega.toFixed(2)} rad/s</span></div>

<div class="mt-2">[2] Dynamic Roller Operating Radius (r_bi):</div>
<div class="pl-4 text-emerald-400/80">r_bi = 30mm + 30mm * Shift_Factor</div>
<div class="pl-4 text-slate-300">r_bi = 30mm + 30mm * ${(p.shiftFactor * 100).toFixed(1)}% = <span class="text-rose-400 font-semibold">${(p.r_bi * 1000).toFixed(1)} mm</span></div>

<div class="mt-2">[3] Total Centrifugal Force of 6 Rollers (F_c):</div>
<div class="pl-4 text-emerald-400/80">Fc = M_total * &omega;² * r_bi</div>
<div class="pl-4 text-slate-300">Fc = (6 * ${p.weight}g) * ${p.omega.toFixed(1)}² * ${p.r_bi.toFixed(3)}m = <span class="text-rose-400 font-semibold">${p.F_c.toFixed(1)} N</span></div>

<div class="mt-2">[4] Front Axial Clamping Force (F_front_axial):</div>
<div class="pl-4 text-emerald-400/80">F_front_axial = (Fc * tan(20°)) / tan(&theta;_front)</div>
<div class="pl-4 text-slate-300">F_front_axial = (${Math.round(p.F_c)} * 0.364) / tan(${p.frontAngleDeg.toFixed(1)}°) = <span class="text-rose-400 font-semibold">${p.F_front_axial.toFixed(1)} N</span></div>

<div class="mt-2 border-t border-amber-900/60 pt-2 font-bold text-amber-400">== SWEET SPOT TUNING RECOMMENDATION (@ ${p.peakRpm} RPM) ==</div>
<div class="text-slate-300 mb-1">Target: Lock shift RPM matching Engine Peak RPM (${p.peakRpm} RPM) for maximum wheel power:</div>

<div class="pl-2 border-l-2 border-amber-500/50 space-y-1">
<div>💡 <span class="text-amber-300 font-bold">Recommended Roller Weight:</span> <span class="text-amber-400 font-extrabold text-sm">${opt.optWeight.toFixed(1)} g</span></div>
<div class="pl-4 text-[11px] text-slate-400">(Based on current ${p.springKg}kg Spring & ${p.frontAngleDeg.toFixed(1)}° Front Pulley Angle)</div>

<div>💡 <span class="text-amber-300 font-bold">Recommended Main Spring Force:</span> <span class="text-amber-400 font-extrabold text-sm">${opt.optSpring.toFixed(1)} kg</span></div>
<div class="pl-4 text-[11px] text-slate-400">(Based on current ${p.weight.toFixed(1)}g Rollers & ${p.frontAngleDeg.toFixed(1)}° Front Pulley Angle)</div>

<div>💡 <span class="text-amber-300 font-bold">Recommended Front Pulley Angle:</span> <span class="text-amber-400 font-extrabold text-sm">${opt.optFrontAngle.toFixed(1)}°</span></div>
<div class="pl-4 text-[11px] text-slate-400">(Based on current ${p.weight.toFixed(1)}g Rollers & ${p.springKg}kg Spring)</div>
</div>

<div class="mt-2 border-t border-emerald-950 pt-2 font-bold text-rose-400">== POTENTIAL POWER LIMITS AT ${p.rpm} RPM ==</div>
<div>[5] Top Speed Limit at ${p.rpm} RPM:</div>
<div class="pl-4 text-rose-300/80">V_max_at_RPM = [ (Current_RPM / (${CONST.GEAR_RATIO_MIN} * Final_Reduction)) * D_wheel * &pi; * 60 ] / 1000</div>
<div class="pl-4 text-slate-300">V_max = [ (${p.rpm} / (${CONST.GEAR_RATIO_MIN} * ${p.finalReduction})) * ${CONST.WHEEL_DIAMETER_M} * 3.1416 * 60 ] / 1000 = <span class="text-rose-400 font-bold">${maxReachableKmhAtSliderRPM.toFixed(1)} km/h</span></div>
<div class="pl-4 text-rose-300/80">X_Cutoff_Percent = (${p.rpm} / ${CONST.MAX_ENGINE_RPM}) * 100%</div>
<div class="pl-4 text-slate-300">X_Cutoff = (${p.rpm} / ${CONST.MAX_ENGINE_RPM}) * 100 = <span class="text-rose-400 font-bold">${maxReachableSpeedPercentAtSliderRPM.toFixed(1)}% Max Speed</span></div>

<div class="mt-2 border-t border-emerald-950 pt-2 font-bold text-amber-400">== LIVE OPERATING POINT EQUATIONS (LIVE TRAJECTORY POINT) ==</div>
<div>[6] Rear Wheel Speed & % of Top Speed (X-Axis):</div>
<div class="pl-4 text-amber-300/80">V_kmh = [ (RPM / (Ratio * Final_Reduction)) * D_wheel * &pi; * 60 ] / 1000</div>
<div class="pl-4 text-slate-300">V_kmh = [ (${p.rpm} / (${p.cvtTransmissionRatio.toFixed(2)} * ${p.finalReduction})) * ${CONST.WHEEL_DIAMETER_M} * 3.1416 * 60 ] / 1000 = <span class="text-amber-400 font-bold">${p.vehicleSpeedKmh.toFixed(1)} km/h</span></div>
<div class="pl-4 text-amber-300/80">X_Percent = (V_kmh / V_max_10k_rpm) * 100%</div>
<div class="pl-4 text-slate-300">X = (${p.vehicleSpeedKmh.toFixed(1)} / ${p.maxSpeedKmh.toFixed(1)}) * 100 = <span class="text-cyan-400 font-bold">${p.speedPercent.toFixed(1)}% Speed</span></div>

<div class="mt-2">[7] Actual Wheel Power Delivered (Y-Axis):</div>
<div class="pl-4 text-amber-300/80">HP_engine = 100 * exp( -0.5 * [ (RPM - Peak_RPM) / Bandwidth ]² )</div>
<div class="pl-4 text-slate-300">HP_engine = 100 * exp( -0.5 * [ (${p.rpm} - ${p.peakRpm}) / ${CONST.ENGINE_BANDWIDTH} ]² ) = <span class="text-slate-200 font-semibold">${p.liveEngineHP.toFixed(1)}% Engine HP</span></div>
<div class="pl-4 text-amber-300/80">Y_Percent = HP_engine * Transmission_Efficiency (&eta;)</div>
<div class="pl-4 text-slate-300">Y = ${p.liveEngineHP.toFixed(1)}% * ${(p.efficiency * 100).toFixed(1)}% = <span class="text-cyan-400 font-bold">${p.liveWheelHP.toFixed(1)}% Wheel HP</span></div>

<div class="mt-2 border-t border-emerald-950 pt-2 font-bold text-slate-300">== REAL-TIME TRANSMISSION TELEMETRY ==</div>
<div class="pl-4 text-slate-300">CVT Ratio (cvtRatio): <span class="text-emerald-400 font-semibold">${p.cvtTransmissionRatio.toFixed(2)} (2.64 -> 0.82)</span></div>
<div class="pl-4 text-slate-300">Shift Travel (Shift Factor): <span class="text-rose-400 font-bold">${(p.shiftFactor * 100).toFixed(1)}%</span></div>
<div class="pl-4 text-slate-300">Front Pulley Radius: <span class="text-emerald-400 font-semibold">${p.R_front_active.toFixed(1)} mm</span> | Rear: <span class="text-emerald-400 font-semibold">${p.R_rear_active.toFixed(1)} mm</span></div>
`;
}

function generateMatlabScript() {
    const matlabOutput = document.getElementById('matlabScriptOutput');
    if (!matlabOutput) {
        return;
    }

    const matlabConstants = Object.entries(CONST)
        .map(([name, value]) => `CONST.${name} = ${Number(value).toPrecision(12)};`)
        .join('\n');
    const rpm = parseFloat(rpmSlider.value);
    const weight = parseFloat(weightSlider.value);
    const springKg = parseFloat(springSlider.value);
    const frontAngleDeg = parseFloat(frontAngleSlider.value);
    const rearAngleDeg = parseFloat(rearAngleSlider.value);
    const peakRpm = parseFloat(peakRpmSlider.value);

    matlabOutput.value = `% CVT Dynamics Simulator - MATLAB export
% Generated: ${new Date().toISOString()}
% This script mirrors calculateDynoCurve() in cvt-physics.js.
clear; clc; close all;

%% System constants
CONST = struct();
${matlabConstants}

%% Current simulator parameters
sliderRPM = ${rpm};
sliderWeight = ${weight};
sliderSpring = ${springKg};
frontAngle = ${frontAngleDeg};
rearAngle = ${rearAngleDeg};
peakRPM = ${peakRpm};
maxSpeedKmh = (CONST.MAX_ENGINE_RPM / (CONST.GEAR_RATIO_MIN * CONST.FINAL_REDUCTION)) * ...
              CONST.WHEEL_DIAMETER_M * pi * 60 / 1000;

%% CVT dyno curve
frontAngleRad = deg2rad(frontAngle);
rearAngleRad = deg2rad(rearAngle);
springFactor = sliderSpring / 40.0;
weightFactor = max(0.1, (sliderWeight * 6) / 114.0);
angleFactor = tan(deg2rad(15.0)) / tan(frontAngleRad);
shiftRPM = 8000 * sqrt(springFactor / weightFactor) * sqrt(angleFactor);
shiftRPM = max(CONST.CLUTCH_ENGAGE_RPM, min(shiftRPM, CONST.MAX_ENGINE_RPM));
maxShiftFactor = min(1.0, max(0.0, (sliderWeight - 2.0) / 7.0));
R_min_actual = CONST.GEAR_RATIO_MAX - ...
    (CONST.GEAR_RATIO_MAX - CONST.GEAR_RATIO_MIN) * maxShiftFactor;
wheelCirc = CONST.WHEEL_DIAMETER_M * pi;
speedAtShiftOutKmh = (shiftRPM / (R_min_actual * CONST.FINAL_REDUCTION)) * ...
    wheelCirc * 60 / 1000;
shiftOutPct = min(90, max(30, (speedAtShiftOutKmh / maxSpeedKmh) * 100));
clutchEngagePct = 15;

pct = (0:100)';
v_kmh = (pct / 100) * maxSpeedKmh;
dynoRPM = zeros(size(pct));
R_cvt = zeros(size(pct));
P_wheel = zeros(size(pct));
T_wheel = zeros(size(pct));

for k = 1:numel(pct)
    p = pct(k);
    v = v_kmh(k);
    reqRPM = (v * 1000 / 60) / wheelCirc * (R_min_actual * CONST.FINAL_REDUCTION);
    clutchFactor = 1.0;
    overdriveLoss = 1.0;

    if p < clutchEngagePct
        phase = p / clutchEngagePct;
        dynoRPM(k) = CONST.CLUTCH_ENGAGE_RPM + phase * ...
            (shiftRPM - CONST.CLUTCH_ENGAGE_RPM);
        clutchFactor = sin(phase * pi / 2);
    elseif p <= shiftOutPct
        phase = (p - clutchEngagePct) / (shiftOutPct - clutchEngagePct);
        dynoRPM(k) = shiftRPM + phase * 250;
    else
        phase = (p - shiftOutPct) / max(100 - shiftOutPct, 1);
        dynoRPM(k) = (shiftRPM + 250) + phase * 2000;
        overdriveLoss = 1.0 - phase * 0.35;
        if shiftRPM < peakRPM - 600
            overdriveLoss = overdriveLoss - phase * 0.3;
        end
        overdriveLoss = max(0.1, overdriveLoss);
    end

    revLimitFactor = 1.0;
    if reqRPM > CONST.MAX_ENGINE_RPM
        revLimitFactor = max(0.0, 1.0 - ...
            (reqRPM - CONST.MAX_ENGINE_RPM) / 1500);
        dynoRPM(k) = CONST.MAX_ENGINE_RPM;
    end

    if p < clutchEngagePct
        R_cvt(k) = CONST.GEAR_RATIO_MAX;
    elseif p <= shiftOutPct
        phase = (p - clutchEngagePct) / (shiftOutPct - clutchEngagePct);
        R_cvt(k) = CONST.GEAR_RATIO_MAX - ...
            (CONST.GEAR_RATIO_MAX - R_min_actual) * phase;
    else
        R_cvt(k) = R_min_actual;
    end

    enginePower = 100 * exp(-0.5 * ((dynoRPM(k) - peakRPM) / ...
        CONST.ENGINE_BANDWIDTH)^2);
    slipDip = 1.0;
    if sliderWeight < 10.0 && p > 15 && p < 60
        dipProgress = (p - 15) / 45;
        dipDepth = (10.0 - sliderWeight) * 0.025;
        slipDip = 1.0 - sin(dipProgress * pi) * dipDepth;
    end
    efficiency = max(0.5, CONST.BASE_CVT_EFFICIENCY - ...
        (abs(frontAngle - 15.0) + abs(rearAngle - 16.0)) * 0.015);
    P_wheel(k) = enginePower * efficiency * clutchFactor * ...
        overdriveLoss * revLimitFactor * slipDip;
    engineTorque = 14.8 * (enginePower / 100) * (peakRPM / max(dynoRPM(k), 1));
    T_wheel(k) = engineTorque * (R_cvt(k) * CONST.FINAL_REDUCTION) * ...
        efficiency * clutchFactor * overdriveLoss * revLimitFactor * slipDip;
end

%% Plot wheel power and torque
figure('Color', 'w', 'Name', 'CVT Dyno Curve');
yyaxis left;
plot(v_kmh, P_wheel, 'Color', [0.00 0.65 0.85], 'LineWidth', 2);
ylabel('Wheel Power (%)');
ylim([0, max(P_wheel) * 1.15]);
yyaxis right;
plot(v_kmh, T_wheel, 'Color', [0.05 0.65 0.45], 'LineWidth', 2);
ylabel('Wheel Torque (Nm)');
xlabel('Vehicle Speed (km/h)');
title(sprintf('CVT Dyno Curve | Shift RPM: %.0f | R_{CVT}: %.2f to %.2f', ...
    shiftRPM, R_cvt(1), R_cvt(end)));
grid on;
legend('Wheel Power', 'Wheel Torque', 'Location', 'best');
`;
}

async function copyMatlabScript() {
    const matlabOutput = document.getElementById('matlabScriptOutput');
    if (!matlabOutput || !matlabOutput.value) {
        alert('MATLAB script is not available yet.');
        return;
    }

    try {
        if (navigator.clipboard && typeof navigator.clipboard.writeText === 'function') {
            await navigator.clipboard.writeText(matlabOutput.value);
        } else {
            matlabOutput.focus();
            matlabOutput.select();
            if (!document.execCommand('copy')) {
                throw new Error('The browser rejected the clipboard fallback.');
            }
            matlabOutput.setSelectionRange(0, 0);
        }
        alert('MATLAB script copied to clipboard.');
    } catch (error) {
        console.error('Unable to copy MATLAB script:', error);
        alert('Unable to copy the MATLAB script. Please copy it manually.');
    }
}

function downloadMatlabScript() {
    const matlabOutput = document.getElementById('matlabScriptOutput');
    if (!matlabOutput || !matlabOutput.value) {
        alert('MATLAB script is not available yet.');
        return;
    }

    const blob = new Blob([matlabOutput.value], { type: 'text/plain;charset=utf-8' });
    const downloadUrl = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = downloadUrl;
    link.download = 'cvt_simulation.m';
    document.body.appendChild(link);
    link.click();
    link.remove();
    URL.revokeObjectURL(downloadUrl);
}

function updatePhysicsAndVisuals() {
    const rpm = parseFloat(rpmSlider.value);
    const weight = parseFloat(weightSlider.value);
    const springKg = parseFloat(springSlider.value);
    const frontAngleDeg = parseFloat(frontAngleSlider.value);
    const rearAngleDeg = parseFloat(rearAngleSlider.value);
    const peakRpm = parseFloat(peakRpmSlider.value);
    CONST.REAL_DYNO_PEAK_RPM = peakRpm;
    document.getElementById('const_peak_rpm').value = peakRpm;

    // Update sliders text
    rpmVal.innerText = `${rpm} RPM`;
    weightVal.innerText = `${weight.toFixed(1)} g`;
    springVal.innerText = `${springKg} kg`;
    frontAngleVal.innerText = `${frontAngleDeg.toFixed(1)}°`;
    rearAngleVal.innerText = `${rearAngleDeg.toFixed(1)}°`;
    peakRpmVal.innerText = `${peakRpm} RPM`;

    // 1. Convert to Base Physics Units
    const m = weight / 1000; // mass per roller (kg)
    const M_total = 6 * m;   // Total 6 rollers (kg)
    const omega = (rpm * 2 * Math.PI) / 60; // angular velocity (rad/s)
    
    // DYNAMIC ROLLER RADIUS FEEDBACK LOOP
    const r_bi = CONST.BASE_ROLLER_RADIUS_M + (CONST.BASE_ROLLER_RADIUS_M * shiftFactor); // m
    
    // Protected against division by zero if weight is 0g
    const F_c = M_total > 0 ? M_total * Math.pow(omega, 2) * r_bi : 0; // Newtons
    
    // Wedge mechanism on Front Pulley
    const theta_ramp_rad = (CONST.RAMP_ANGLE_DEG * Math.PI) / 180;
    const F_centrifugalForceToAxial = F_c * Math.tan(theta_ramp_rad); // N
    
    // Front pulley sliding mechanical advantage
    const frontAngleRad = (frontAngleDeg * Math.PI) / 180;
    const F_front_axial = F_centrifugalForceToAxial / Math.tan(frontAngleRad); // N

    // DYNAMIC REAR SPRING COMPRESSION FEEDBACK LOOP
    const activeSpringKg = springKg * (0.8 + 0.4 * shiftFactor);
    const F_rear_axial_spring = activeSpringKg * 9.81; // N
    const rearAngleRad = (rearAngleDeg * Math.PI) / 180;
    
    // Radial belt tension wedge component
    const F_rear_belt_tension = F_rear_axial_spring / Math.tan(rearAngleRad); // N

    // 2. Compute Equilibrium and Belt Shift Factor
    const forceRatio = F_front_axial / (F_rear_belt_tension || 1);
    
    // Dynamic non-linear mapped shift factor
    targetShiftFactor = (forceRatio - 0.5) / 1.5;
    targetShiftFactor = Math.min(Math.max(targetShiftFactor, 0), 1);

    // 3. Map Belt Positions
    const R_front_min = 25; // fixed boss radius in mm
    const R_front_max = frontMaxD / 2;
    const R_rear_min = 35;  // fixed minimum radius for real belt clearance
    const R_rear_max = rearMaxD / 2;

    // REAL-WORLD PHYSICAL SPECIFICATIONS (HONDA SH160 TRUYỀN ĐỘNG)
    const cvtTransmissionRatio = CONST.GEAR_RATIO_MAX - (CONST.GEAR_RATIO_MAX - CONST.GEAR_RATIO_MIN) * shiftFactor;
    // Use the same ratio for the visual belt pitch radii so the drawing and telemetry agree.
    const R_front_active = R_front_min + (R_front_max - R_front_min) * shiftFactor;
    const R_rear_active = Math.min(R_rear_max, Math.max(R_rear_min, R_front_active * cvtTransmissionRatio));
    const finalReduction = CONST.FINAL_REDUCTION;
    const wheelDiameterM = CONST.WHEEL_DIAMETER_M;

    const wheelRPM = rpm / (cvtTransmissionRatio * finalReduction);
    const vehicleSpeedKmh = wheelRPM * wheelDiameterM * Math.PI * 60 / 1000;

    // Calculate Max Theoretical Speed at Max RPM, kịch số overdrive
    const maxSpeedKmh = (CONST.MAX_ENGINE_RPM / (CONST.GEAR_RATIO_MIN * finalReduction)) * wheelDiameterM * Math.PI * 60 / 1000;
    const speedPercent = (vehicleSpeedKmh / maxSpeedKmh) * 100;

    // Update UI indicators
    ratioDisplay.innerText = `${cvtTransmissionRatio.toFixed(2)}`;
    shiftDisplay.innerText = `${(shiftFactor * 100).toFixed(1)}%`;

    // Update status banner
    updateStatusText(rpm, shiftFactor, peakRpm);

    // HP Curve is modeled as a realistic Stage 1 curve
    // DYNAMIC TRANSMISSION EFFICIENCY CALCULATION
    const liveEngineHP = calculateEnginePowerPercent(rpm, peakRpm);
    let efficiency = CONST.BASE_CVT_EFFICIENCY;
    
    // 1. Clutch slip below 12% speed
    if (speedPercent < 12) {
        efficiency *= (speedPercent / 12);
    }
    // 2. High RPM belt slip and thermal loss
    if (rpm > 8500) {
        const excessRpm = rpm - 8500;
        efficiency -= (excessRpm / 1500) * 0.10;
    }
    efficiency = Math.max(0, Math.min(efficiency, 0.85));
    const liveWheelHP = liveEngineHP * efficiency;

    // Compute Optimal Setup Recommendation
    const optTuning = calculateOptimalTuning(peakRpm, weight, springKg, frontAngleDeg, rearAngleDeg);

    // Render visuals (These are defined in cvt-canvas.js)
    if (typeof drawCVT === 'function') {
        drawCVT(R_front_active, R_rear_active, R_front_max, R_rear_max, rpm);
    }
    
    // Render Top Dyno Chart with all embedded curves
    if (typeof drawDynoChart === 'function') {
        const { speedPoints, trajectoryPoints } = calculateCurvePoints(speedPercent, liveWheelHP, rpm, weight, springKg, frontAngleDeg, rearAngleDeg, maxSpeedKmh, peakRpm);
        drawDynoChart(speedPercent, liveWheelHP, vehicleSpeedKmh, weight, springKg, frontAngleDeg, rearAngleDeg, maxSpeedKmh, peakRpm, speedPoints, trajectoryPoints);
    }

    // Chỉ render lại debugConsole khi thật sự cần hoặc khi không có cửa sổ prompt
    updateDebugLog({
        rpm, weight, springKg, activeSpringKg, frontAngleDeg, rearAngleDeg,
        totalMass: M_total, omega, r_bi, F_c, F_centrifugalForceToAxial, F_front_axial,
        F_rear_axial_spring, F_rear_belt_tension, forceRatio, shiftFactor, targetShiftFactor,
        R_front_active, R_rear_active, cvtTransmissionRatio, finalReduction, speedPercent,
        vehicleSpeedKmh, maxSpeedKmh, liveEngineHP, efficiency, liveWheelHP, peakRpm,
        optTuning
    });
    generateMatlabScript();
}