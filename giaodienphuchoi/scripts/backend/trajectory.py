from abc import ABC, abstractmethod
import math

class TrajectoryBase(ABC):
    def __init__(self):
        # Khởi tạo các biến lưu trữ tham số đã tính toán
        self.start_p = -90.0
        self.end_p = -90.0
        self.maxVel = math.inf
        self.direction = 1.0
        self.total_time = math.inf 
        
    @abstractmethod
    def param_calc(self, start_p, end_p, max_v):
        """
        Method này chạy NẶNG, chứa logic phức tạp (căn bậc 2, giải phương trình...).
        Chỉ gọi 1 lần khi có lệnh Move mới.
        """
        pass

    @abstractmethod
    def desired_state(self, t):
        """
        Method này chạy NHẸ (chỉ cộng trừ nhân chia đơn giản).
        Gọi liên tục trong vòng lặp timer (Real-time).
        Output: pos, vel, acc
        """
        pass
    def reset(self):
        self.start_p = -90.0
        self.end_p = -90.0
        self.total_time = math.inf
        self.direction = 1.0 

class TrapezoidalTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Các biến riêng của hình thang
        self.j_peak = 4   # deg/s^3
        self.accel = 10.0  # deg/s^2 - gia tốc cố định
        self.v_peak = 0.0 # deg/s
        self.t_acc = 0.0  # Thời gian tăng tốc
        self.t_dec = 0.0  # Thời điểm bắt đầu giảm tốc

    def param_calc(self, start_p, end_p, max_v):
        self.start_p = start_p
        self.end_p = end_p
        distance = end_p - start_p
        self.direction = 1.0 if distance >= 0 else -1.0
        abs_dist = abs(distance)
        
        # Safety check: max_v must be positive
        if max_v <= 0:
            print(f"[Warning] TrapezoidalTrajectory: max_v={max_v} is not positive, using default 1.0")
            max_v = 1.0
        
        if abs_dist < 0.005:
            self.v_peak = 0.0
            self.t_acc = 0.0
            self.total_time = 0.0
            self.t_dec = 0.0
            return
        
        # Tính thời gian tăng tốc
        self.t_acc = max_v / self.accel
        d_acc = 0.5 * self.accel * self.t_acc ** 2
        
        if abs_dist <= 2 * d_acc:
            # Không có giai đoạn tốc độ đều
            # Safety: Ensure abs_dist * self.accel >= 0 before sqrt
            if abs_dist * self.accel < 0:
                print(f"[Error] TrapezoidalTrajectory: abs_dist={abs_dist}, accel={self.accel}, product is negative!")
                self.v_peak = 0.0
            else:
                self.v_peak = math.sqrt(abs_dist * self.accel)
            self.t_acc = self.v_peak / self.accel if self.accel > 0 else 1.0
            self.total_time = 2 * self.t_acc
            self.t_dec = self.t_acc
        else:
            # Có giai đoạn tốc độ đều
            self.v_peak = max_v
            d_const = abs_dist - 2 * d_acc
            t_const = d_const / max_v if max_v > 0 else 0.0
            self.total_time = 2 * self.t_acc + t_const
            self.t_dec = self.t_acc + t_const


    def desired_state(self, t):
        # Xử lý ngoài phạm vi

        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time or self.total_time == math.inf: return self.end_p, 0.0, 0.0

        pos = 0.0
        vel = 0.0
        acc = 0.0   

        if t < self.t_acc: # Giai đoạn 1: Tăng tốc
            pos = 0.5 * self.accel * t * t
            vel = self.accel * t
            acc = self.accel
            
        elif t < self.t_dec: # Giai đoạn 2: Tốc độ đều
            dt = t - self.t_acc
            pos = 0.5 * self.accel * self.t_acc**2 + self.v_peak * dt
            vel = self.v_peak
            acc = 0.0
            
        else: # Giai đoạn 3: Giảm tốc
            t_rem = self.total_time - t
            dist_rem = 0.5 * self.accel * t_rem * t_rem
            pos = abs(self.end_p - self.start_p) - dist_rem
            vel = self.accel * t_rem
            acc = -self.accel

        # Kết hợp hướng
        final_p = self.start_p + pos * self.direction
        final_v = vel * self.direction
        final_a = acc * self.direction
        
        return final_p, final_v, final_a
    
class CubicTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Hệ số phương trình: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0

    def param_calc(self, start_p, end_p, max_v):
        
        self.start_p = start_p
        self.end_p = end_p
        dist = end_p - start_p
        abs_dist = abs(dist)
        
        # Tính thời gian dựa trên max_v, giả sử thời gian = abs_dist / max_v * 2 để smooth và không vượt quá max_v quá nhiều
        self.total_time = abs_dist / max_v * 2.0 if max_v > 0 else 1.0
        
        T = self.total_time
        self.a0 = start_p
        self.a1 = 0
        self.a2 = 3 * dist / (T**2)
        self.a3 = -2 * dist / (T**3)

    def desired_state(self, t):
        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time or self.total_time == math.inf: return self.end_p, 0.0, 0.0
        
        t2 = t*t
        t3 = t2*t
        
        pos = self.a0 + self.a1*t + self.a2*t2 + self.a3*t3
        vel = self.a1 + 2*self.a2*t + 3*self.a3*t2
        acc = 2*self.a2 + 6*self.a3*t
        
        return pos, vel, acc
    
class QuinticTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Hệ số phương trình bậc 5: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.a5 = 0

    def param_calc(self, start_p, end_p, max_v):
        self.start_p = start_p
        self.end_p = end_p
        dist = end_p - start_p
        
        # Tính thời gian dựa trên max_v, giả sử thời gian = abs(dist) / max_v * 2.5 để smooth
        abs_dist = abs(dist)
        self.total_time = abs_dist / max_v * 2.5 if max_v > 0 else 1.0
        
        T = self.total_time
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T
        
        self.a0 = start_p
        self.a1 = 0
        self.a2 = 0
        self.a3 = 10 * dist / T3
        self.a4 = -15 * dist / T4
        self.a5 = 6 * dist / T5

    def desired_state(self, t):
        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time or self.total_time == math.inf: return self.end_p, 0.0, 0.0
        
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        
        pos = self.a0 + self.a1*t + self.a2*t2 + self.a3*t3 + self.a4*t4 + self.a5*t5
        vel = self.a1 + 2*self.a2*t + 3*self.a3*t2 + 4*self.a4*t3 + 5*self.a5*t4
        acc = 2*self.a2 + 6*self.a3*t + 12*self.a4*t2 + 20*self.a5*t3
        
        return pos, vel, acc
    
class SplineTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Các tham số cho quỹ đạo splines 7 đoạn với giới hạn jerk
        self.max_jerk = 400  # deg/s^3
        self.max_acc = 80  # deg/s^2
        # Các thời gian cho từng phase
        self.t1 = 0.0
        self.t2 = 0.0
        self.t3 = 0.0
        self.t4 = 0.0
        self.t5 = 0.0
        self.t6 = 0.0
        self.t7 = 0.0
        # Các khoảng cách tích lũy (tùy chọn, để debug)
        self.d1 = 0.0
        self.d2 = 0.0
        self.d3 = 0.0
        self.d4 = 0.0
        self.d5 = 0.0
        self.d6 = 0.0
        self.d7 = 0.0

    def param_calc(self, start_p, end_p, max_v):
        self.start_p = start_p
        self.end_p = end_p
        dist = end_p - start_p
        abs_dist = abs(dist)
        self.direction = 1.0 if dist >= 0 else -1.0
        
        # Safety check: max_v must be positive
        if max_v <= 0:
            print(f"[Warning] SplineTrajectory: max_v={max_v} is not positive, using default 1.0")
            max_v = 1.0
        
        # Tính thời gian jerk phase
        t_j = self.max_acc / self.max_jerk
        d_j = (1/6.0) * self.max_jerk * t_j**3
        d_a = self.max_acc * t_j**2
        d_acc = 2 * d_j + d_a
        
        if abs_dist > 2 * d_acc:
            t_const = (abs_dist - 2 * d_acc) / max_v
        else:
            # Điều chỉnh nếu không đủ khoảng cách cho constant vel
            # Safety: Check d_acc > 0 before sqrt
            numerator = abs_dist / (2 * d_acc) if d_acc > 0 else 1.0
            
            if numerator < 0:
                print(f"[Error] SplineTrajectory: numerator={numerator} is negative, using 1.0")
                scale = 1.0
            else:
                scale = math.sqrt(numerator)
            
            t_j *= scale
            d_j *= scale**3
            d_a *= scale**2
            d_acc = 2 * d_j + d_a
            t_const = 0.0
        
        self.t1 = t_j
        self.t2 = t_j
        self.t3 = t_j
        self.t4 = t_const
        self.t5 = t_j
        self.t6 = t_j
        self.t7 = t_j
        self.total_time = self.t1 + self.t2 + self.t3 + self.t4 + self.t5 + self.t6 + self.t7
        
        # Tính khoảng cách tích lũy tại cuối mỗi phase
        self.d1 = (1/6.0) * self.max_jerk * self.t1**3
        self.d2 = self.d1 + 0.5 * self.max_acc * self.t1**2 + self.max_acc * self.t1 * self.t2 + 0.5 * self.max_acc * self.t2**2
        self.d3 = self.d2 + self.max_acc * self.t2 * self.t3 + 0.5 * self.max_acc * self.t3**2 - (1/6.0) * self.max_jerk * self.t3**3
        self.d4 = self.d3 + self.max_acc * self.t2 * self.t4
        self.d5 = self.d4 + self.max_acc * self.t2 * self.t5 - (1/6.0) * self.max_jerk * self.t5**3
        self.d6 = self.d5 + (self.max_acc * self.t2 - self.max_acc * self.t5) * self.t6 - 0.5 * self.max_acc * self.t6**2
        self.d7 = self.d6 + (self.max_acc * self.t2 - self.max_acc * self.t6) * self.t7 - 0.5 * self.max_acc * self.t7**2 + (1/6.0) * self.max_jerk * self.t7**3

    def desired_state(self, t):
        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time or self.total_time == math.inf: return self.end_p, 0.0, 0.0
        
        pos = 0.0
        vel = 0.0
        acc = 0.0
        jerk = 0.0
        
        if t < self.t1:
            # Phase 1: jerk = max_jerk, acc tăng từ 0
            jerk = self.max_jerk
            acc = jerk * t
            vel = 0.5 * jerk * t**2
            pos = (1/6.0) * jerk * t**3
        elif t < self.t1 + self.t2:
            # Phase 2: jerk = 0, acc = max_acc
            dt = t - self.t1
            jerk = 0.0
            acc = self.max_acc
            vel = 0.5 * self.max_jerk * self.t1**2 + acc * dt
            pos = (1/6.0) * self.max_jerk * self.t1**3 + 0.5 * acc * self.t1**2 + acc * self.t1 * dt + 0.5 * acc * dt**2
        elif t < self.t1 + self.t2 + self.t3:
            # Phase 3: jerk = -max_jerk, acc giảm từ max_acc về 0
            dt = t - (self.t1 + self.t2)
            jerk = -self.max_jerk
            acc = self.max_acc + jerk * dt
            vel = 0.5 * self.max_jerk * self.t1**2 + self.max_acc * self.t2 + self.max_acc * dt + 0.5 * jerk * dt**2
            pos = (1/6.0) * self.max_jerk * self.t1**3 + 0.5 * self.max_acc * self.t1**2 + self.max_acc * self.t1 * self.t2 + 0.5 * self.max_acc * self.t2**2 + self.max_acc * self.t2 * dt + 0.5 * self.max_acc * dt**2 + (1/6.0) * jerk * dt**3
        elif t < self.t1 + self.t2 + self.t3 + self.t4:
            # Phase 4: jerk = 0, acc = 0, vel constant
            dt = t - (self.t1 + self.t2 + self.t3)
            jerk = 0.0
            acc = 0.0
            vel = self.max_acc * self.t2  # vel at end of phase 2
            pos = self.d3 + vel * dt
        elif t < self.t1 + self.t2 + self.t3 + self.t4 + self.t5:
            # Phase 5: jerk = -max_jerk, acc giảm từ 0 về -max_acc
            dt = t - (self.t1 + self.t2 + self.t3 + self.t4)
            jerk = -self.max_jerk
            acc = jerk * dt
            vel = self.max_acc * self.t2 - 0.5 * self.max_jerk * dt**2
            pos = self.d4 + self.max_acc * self.t2 * dt - (1/6.0) * self.max_jerk * dt**3
            # d4 = d3 + vel * self.t4
        elif t < self.t1 + self.t2 + self.t3 + self.t4 + self.t5 + self.t6:
            # Phase 6: jerk = 0, acc = -max_acc
            dt = t - (self.t1 + self.t2 + self.t3 + self.t4 + self.t5)
            jerk = 0.0
            acc = -self.max_acc
            vel = self.max_acc * self.t2 - self.max_acc * dt
            pos = self.d5 + self.max_acc * self.t2 * dt - 0.5 * self.max_acc * dt**2
        else:
            # Phase 7: jerk = max_jerk, acc tăng từ -max_acc về 0
            dt = t - (self.t1 + self.t2 + self.t3 + self.t4 + self.t5 + self.t6)
            jerk = self.max_jerk
            acc = -self.max_acc + jerk * dt
            vel = self.max_acc * self.t2 - self.max_acc * self.t6 + -self.max_acc * dt + 0.5 * jerk * dt**2
            pos = self.d6 + (self.max_acc * self.t2 - self.max_acc * self.t6) * dt - 0.5 * self.max_acc * self.t6 * dt + -0.5 * self.max_acc * dt**2 + (1/6.0) * jerk * dt**3
        
        # Điều chỉnh cho hướng
        final_pos = self.start_p + pos * self.direction
        final_vel = vel * self.direction
        final_acc = acc * self.direction
        final_jerk = jerk * self.direction
        
        return final_pos, final_vel, final_acc


class SinusoidalTrajectory(TrajectoryBase):
    """
    Continuous sinusoidal oscillation trajectory.
    Oscillates continuously between start_p and end_p until trajectory is stopped externally.
    
    Parameters:
    - amplitude: (end_p - start_p) / 2
    - center: (start_p + end_p) / 2
    - omega (angular frequency): max_v / amplitude (if amplitude > 0)
    - Period T = 2π / omega (one complete oscillation cycle)
    
    Equations:
    - pos(t) = center + amplitude * sin(ωt)
    - vel(t) = amplitude * ω * cos(ωt)
    - acc(t) = -amplitude * ω² * sin(ωt)
    
    The oscillation continues indefinitely until external stop (reset() or return_IDLE())
    """
    def __init__(self):
        super().__init__()
        self.center = 0.0      # Center position
        self.amplitude = 0.0   # Oscillation amplitude
        self.omega = 0.0       # Angular frequency (rad/s)

    def param_calc(self, start_p, end_p, max_v):
        """
        Calculate sinusoidal trajectory parameters.
        
        Args:
            start_p: Starting position (will oscillate between this and end_p)
            end_p: Ending position (will oscillate between start_p and this)
            max_v: Maximum velocity (used to calculate oscillation frequency)
        """
        self.start_p = start_p
        self.end_p = end_p
        
        # Safety check: max_v must be positive
        if max_v <= 0:
            print(f"[Warning] SinusoidalTrajectory: max_v={max_v} is not positive, using default 1.0")
            max_v = 1.0
        
        # Calculate amplitude and center
        self.amplitude = (end_p - start_p) / 2.0
        self.center = (start_p + end_p) / 2.0
        
        # Handle edge case: start_p == end_p (no oscillation)
        if abs(self.amplitude) < 0.001:
            self.amplitude = 0.0
            self.omega = 0.0
            self.total_time = float('inf')  # Infinite time for continuous oscillation
            return
        
        # Calculate angular frequency from max velocity
        # max_velocity of sin(ωt) is ω * amplitude
        # so ω = max_v / amplitude
        # Safety: Check amplitude != 0 before division
        if abs(self.amplitude) == 0:
            self.omega = 0.0
            print(f"[Warning] SinusoidalTrajectory: amplitude is 0 after check, setting omega=0")
        else:
            self.omega = max_v / abs(self.amplitude)
        
        # Period of one oscillation cycle: T = 2π / ω
        # But total_time is set to inf to allow continuous oscillation
        self.total_time = float('inf')

    def desired_state(self, t):
        """
        Get desired position, velocity, and acceleration at time t.
        
        Returns:
            (pos, vel, acc): Position (deg), velocity (deg/s), acceleration (deg/s²)
        
        Note: Oscillation continues indefinitely. Use reset() or return_IDLE() to stop.
        """
        if t <= 0:
            return self.start_p, 0.0, 0.0
        
        if self.amplitude == 0.0:
            return self.start_p, 0.0, 0.0
        
        # Calculate sinusoidal trajectory - continuous oscillation
        # No stopping at total_time, oscillates indefinitely
        sine_term = math.sin(self.omega * t - math.pi/2)
        cos_term = math.cos(self.omega * t - math.pi/2)
        omega2 = self.omega ** 2
        
        # Position: center + amplitude * sin(ωt)
        pos = self.center + self.amplitude * sine_term
        
        # Velocity: amplitude * ω * cos(ωt)
        vel = self.amplitude * self.omega * cos_term
        
        # Acceleration: -amplitude * ω² * sin(ωt)
        acc = -self.amplitude * omega2 * sine_term
        
        return pos, vel, acc