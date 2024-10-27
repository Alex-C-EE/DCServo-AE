import math

class DCMotor:
    def __init__(self):
        # Motor specifications
        self.V_nominal = 6.0  # Nominal voltage (Volts)
        self.speed_no_load_rpm = 140  # No-load speed (RPM)
        self.I_no_load = 0.14  # No-load current (Amperes)
        self.I_stall = 2.9  # Stall current (Amperes)
        self.T_stall_kgmm = 54  # Stall torque (kg·mm)
        
        # Linear coefficients from the given relationships
        self.k_speed = (self.speed_no_load_rpm) / self.T_stall_kgmm  # rpm per kg·mm
        self.k_current = (self.I_stall - self.I_no_load) / self.T_stall_kgmm  # A per kg·mm
    
    def compute_performance(self, T_load_kgmm):
        # Ensure torque is within valid range
        if T_load_kgmm < 0 or T_load_kgmm > self.T_stall_kgmm:
            raise ValueError("Torque must be between 0 and stall torque (54 kg·mm).")
        
        # Speed calculation (rpm)
        speed_rpm = self.speed_no_load_rpm - self.k_speed * T_load_kgmm
        
        # Current calculation (A)
        current_A = self.I_no_load + self.k_current * T_load_kgmm
        
        # Convert torque to N·m
        T_load_Nm = T_load_kgmm * 9.80665e-3  # kg·mm to N·m
        
        # Convert speed to rad/s
        omega_rad_per_sec = speed_rpm * (2 * math.pi / 60)
        
        # Mechanical output power (W)
        P_out_W = T_load_Nm * omega_rad_per_sec
        
        # Electrical input power (W)
        P_in_W = self.V_nominal * current_A
        
        # Efficiency (%)
        efficiency = (P_out_W / P_in_W) * 100 if P_in_W > 0 else 0
        
        return {
            'torque_kgmm': T_load_kgmm,
            'torque_Nm': T_load_Nm,
            'speed_rpm': speed_rpm,
            'current_A': current_A,
            'output_power_W': P_out_W,
            'input_power_W': P_in_W,
            'efficiency_percent': efficiency
        }

motor = DCMotor()

# Stall torque
T_load_kgmm = 53  # kg·mm

performance = motor.compute_performance(T_load_kgmm)

print(f"Speed (rpm): {performance['speed_rpm']:.2f}")
print(f"Current (A): {performance['current_A']:.2f}")
print(f"Output Power (W): {performance['output_power_W']:.2f}")
print(f"Efficiency (%): {performance['efficiency_percent']:.2f}")

