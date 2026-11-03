"""Delta robot inverse and forward kinematics."""

import math

sqrt3 = math.sqrt(3)
sin120 = sqrt3 / 2.0
cos120 = -0.5


class DeltaKinematics:
    def __init__(self, f, e, rf, re):
        """
        f  = base equilateral triangle side length (mm)
        e  = end effector equilateral triangle side length (mm)
        rf = upper arm length, motor shaft to elbow (mm)
        re = lower arm length, elbow to end effector (mm)
        """
        self.f = f
        self.e = e
        self.rf = rf
        self.re = re
        # half side / sqrt(3) = distance from center to side midpoint
        self.y1 = -f / (2.0 * sqrt3)
        self.y_offset = e / (2.0 * sqrt3)

    def inverse(self, x0, y0, z0):
        """Inverse kinematics: (x, y, z) -> (theta1, theta2, theta3) in degrees.

        Returns three motor angles. Raises ValueError if position is unreachable.
        """
        theta1 = self._angle_yz(x0, y0, z0)
        theta2 = self._angle_yz(
            x0 * cos120 + y0 * sin120,
            y0 * cos120 - x0 * sin120,
            z0,
        )
        theta3 = self._angle_yz(
            x0 * cos120 - y0 * sin120,
            y0 * cos120 + x0 * sin120,
            z0,
        )
        return (theta1, theta2, theta3)

    def _angle_yz(self, x0, y0, z0):
        y0 -= self.y_offset
        a = (x0 * x0 + y0 * y0 + z0 * z0 + self.rf * self.rf
             - self.re * self.re - self.y1 * self.y1) / (2.0 * z0)
        b = (self.y1 - y0) / z0
        d = -(a + b * self.y1) * (a + b * self.y1) + self.rf * self.rf * (b * b + 1.0)
        if d < 0:
            raise ValueError("Unreachable position")
        yj = (self.y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
        zj = a + b * yj
        return math.atan2(-zj, self.y1 - yj) * 180.0 / math.pi

    def forward(self, theta1, theta2, theta3):
        """Forward kinematics: (theta1, theta2, theta3) in degrees -> (x, y, z).

        Returns end effector position. Raises ValueError if no solution.
        """
        t1 = math.radians(theta1)
        t2 = math.radians(theta2)
        t3 = math.radians(theta3)

        # Motor 1
        y1 = -(self.f / (2.0 * sqrt3) - self.e / (2.0 * sqrt3))
        z1 = -self.rf * math.sin(t1)
        hy1 = y1 - self.rf * math.cos(t1)

        # Motor 2
        y2 = (self.f / (2.0 * sqrt3) - self.e / (2.0 * sqrt3)) / 2.0
        x2 = y2 * sqrt3
        z2 = -self.rf * math.sin(t2)
        hy2_base = self.rf * math.cos(t2)
        hx2 = (self.f / (2.0 * sqrt3) - self.e / (2.0 * sqrt3)) * sin120 + hy2_base * sin120
        hy2 = -(self.f / (2.0 * sqrt3) - self.e / (2.0 * sqrt3)) * cos120 - hy2_base * cos120

        # Motor 3
        z3 = -self.rf * math.sin(t3)
        hy3_base = self.rf * math.cos(t3)
        hx3 = -(self.f / (2.0 * sqrt3) - self.e / (2.0 * sqrt3)) * sin120 - hy3_base * sin120
        hy3 = -(self.f / (2.0 * sqrt3) - self.e / (2.0 * sqrt3)) * cos120 - hy3_base * cos120

        # Solve for x, y, z from three sphere equations
        dnm = (hy2 - hy1) * hx3 - (hy3 - hy1) * hx2

        w1 = hy1 * hy1 + z1 * z1
        w2 = hx2 * hx2 + hy2 * hy2 + z2 * z2
        w3 = hx3 * hx3 + hy3 * hy3 + z3 * z3

        a1 = (z2 - z1) * (hy3 - hy1) - (z3 - z1) * (hy2 - hy1)
        b1 = -((w2 - w1) * (hy3 - hy1) - (w3 - w1) * (hy2 - hy1)) / 2.0

        a2 = -(z2 - z1) * hx3 + (z3 - z1) * hx2
        b2 = ((w2 - w1) * hx3 - (w3 - w1) * hx2) / 2.0

        a = a1 * a1 + a2 * a2 + dnm * dnm
        b_coeff = 2.0 * (a1 * b1 + a2 * (b2 - hy1 * dnm) - z1 * dnm * dnm)
        c = (b1 * b1 + (b2 - hy1 * dnm) * (b2 - hy1 * dnm)
             + dnm * dnm * (z1 * z1 - self.re * self.re))

        d = b_coeff * b_coeff - 4.0 * a * c
        if d < 0:
            raise ValueError("No FK solution")

        z0 = -0.5 * (b_coeff + math.sqrt(d)) / a
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm

        return (x0, y0, z0)
