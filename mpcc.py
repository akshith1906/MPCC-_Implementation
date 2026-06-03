import casadi as ca
import numpy as np

class MPCC:
    def __init__(self, horizon=25, dt=0.1): # Horizon updated to 25 to match simulation
        self.N = horizon
        self.dt = dt

        self.nx = 5
        self.nu = 3
        self.L = 0.33
        
        # Adjusted weights for smooth track following
        self.q_c = 2.0
        self.q_l = 10.0 
        self.q_progress = 50.0

        self.r_a = 0.1
        self.r_delta = 0.1
        self.r_vtheta = 0.1

        self.build_optimizer()

    def build_optimizer(self):
        N = self.N
        dt = self.dt

        opti = ca.Opti()

        X = opti.variable(self.nx, N + 1)
        U = opti.variable(self.nu, N)

        x = X[0, :]
        y = X[1, :]
        psi = X[2, :]
        v = X[3, :]
        theta = X[4, :]

        a = U[0, :]
        delta = U[1, :]
        v_theta = U[2, :]

        self.X = X
        self.U = U

        X0 = opti.parameter(self.nx)
        self.X0 = X0

        # Obstacle parameters (Configuration Space circles: [cx, cy, total_radius])
        self.StaticObs = opti.parameter(3)
        self.DynObs = opti.parameter(3, N)

        slack_s = opti.variable(1,N)
        slack_d = opti.variable(1,N)

        cost = 0

        for k in range(N):
            x_next = x[k] + dt * v[k] * ca.cos(psi[k])
            y_next = y[k] + dt * v[k] * ca.sin(psi[k])
            psi_next = psi[k] + dt * (v[k] / self.L) * ca.tan(delta[k])
            v_next = v[k] + dt * a[k]
            theta_next = theta[k] + dt * v_theta[k]

            opti.subject_to(X[:, k + 1] == ca.vertcat(
                x_next, y_next, psi_next, v_next, theta_next
            ))

            ref_x = 5 * ca.cos(theta[k])
            ref_y = 5 * ca.sin(theta[k])

            tangent = ca.vertcat(-ca.sin(theta[k]), ca.cos(theta[k]))
            normal = ca.vertcat(-ca.cos(theta[k]), -ca.sin(theta[k]))

            error = ca.vertcat(x[k] - ref_x, y[k] - ref_y)

            e_c = ca.dot(normal, error)
            e_l = ca.dot(tangent, error)
            
            # Heading penalty to prevent donuts/driving backwards
            e_heading = psi[k] - (theta[k] + ca.pi / 2)
            q_heading = 2.0 

            stage_cost = (
                self.q_c * e_c**2 +
                self.q_l * e_l**2 +
                q_heading * e_heading**2 -
                self.q_progress * v_theta[k] +
                self.r_a * a[k]**2 +
                self.r_delta * delta[k]**2 +
                self.r_vtheta * v_theta[k]**2 +
                500 * (slack_s[k]**2 + slack_d[k]**2) # Heavy penalty for constraint violation
            )

            cost += stage_cost

# --- Soft Circular C-Space Collision Avoidance ---
            # 1. Static Obstacle
            dist_sq_s = (x[k] - self.StaticObs[0])**2 + (y[k] - self.StaticObs[1])**2
            opti.subject_to(dist_sq_s + slack_s[k] >= self.StaticObs[2]**2)
            opti.subject_to(slack_s[k] >= 0) # Slack cannot be negative

            # 2. Dynamic Obstacle
            dist_sq_d = (x[k] - self.DynObs[0, k])**2 + (y[k] - self.DynObs[1, k])**2
            opti.subject_to(dist_sq_d + slack_d[k] >= self.DynObs[2, k]**2)
            opti.subject_to(slack_d[k] >= 0)

        # Penalize rate of change to ensure smooth driving
        r_ddelta = 50.0
        r_da = 1.0

        for k in range(N - 1):
            cost += r_ddelta * (delta[k + 1] - delta[k])**2
            cost += r_da * (a[k + 1] - a[k])**2

        opti.minimize(cost)

        opti.subject_to(X[:, 0] == X0)

        opti.subject_to(opti.bounded(-2.0, a, 2.0))
        opti.subject_to(opti.bounded(-0.8, delta, 0.8))
        opti.subject_to(opti.bounded(0.0, v, 5.0))
        opti.subject_to(opti.bounded(0.0, v_theta, 1.0)) # Capped to prevent lap desync

        opts = {
            "ipopt.print_level": 0,
            "print_time": 0,
        }

        opti.solver("ipopt", opts)
        self.opti = opti

    def solve(self, current_state, static_obs, dyn_obs_traj):
        self.opti.set_value(self.X0, current_state)
        self.opti.set_value(self.StaticObs, static_obs)
        self.opti.set_value(self.DynObs, dyn_obs_traj)

        # --- ADD THIS: Warm Starting ---
        # If we have a previous solution, use it as the starting guess
        # to prevent the solver from flip-flopping and chattering!
        if hasattr(self, 'last_X'):
            self.opti.set_initial(self.X, self.last_X)
            self.opti.set_initial(self.U, self.last_U)
        # -------------------------------

        sol = self.opti.solve()

        # --- ADD THIS: Save the solution for next time ---
        self.last_X = sol.value(self.X)
        self.last_U = sol.value(self.U)
        # -------------------------------------------------

        u = sol.value(self.U[:, 0])
        predicted_states = sol.value(self.X)

        return u, predicted_states