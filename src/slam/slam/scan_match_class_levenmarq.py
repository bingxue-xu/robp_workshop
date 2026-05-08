from scipy.optimize import least_squares
import numpy as np


class ScanMatch:
    def __init__(self, source, target, batch_size=1000):
        self.source = source
        self.target = target
        self.batch_size = batch_size

    def transform_points(self, R, t, points):
        transformed_points = np.dot(R, points.T).T + t
        return transformed_points

    def residual_function(self, params, batch_points):
        # Params contains the transformation parameters (rotation and translation)
        R = self.rotation_matrix(params[:3])
        t = params[3:]

        transformed_points = self.transform_points(R, t, batch_points)
        residuals = transformed_points - self.target
        return residuals.flatten()

    def rotation_matrix(self, angles):
        theta_x, theta_y, theta_z = angles
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(theta_x), -np.sin(theta_x)],
                       [0, np.sin(theta_x), np.cos(theta_x)]])
        Ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                       [0, 1, 0],
                       [-np.sin(theta_y), 0, np.cos(theta_y)]])
        Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                       [np.sin(theta_z), np.cos(theta_z), 0],
                       [0, 0, 1]])
        return np.dot(Rz, np.dot(Ry, Rx))

    def optimize(self, initial_guess):
        num_batches = len(self.source) // self.batch_size
        if len(self.source) % self.batch_size != 0:
            pass
            # num_batches += 1

        R_optimized = None
        t_optimized = None

        for i in range(num_batches-1):
            batch_source = self.source[i*self.batch_size:(i+1)*self.batch_size]
            def batch_residual_function(
                params): return self.residual_function(params, batch_source)
            result = least_squares(batch_residual_function, initial_guess)
            optimized_params = result.x

            R_batch = self.rotation_matrix(optimized_params[:3])
            t_batch = optimized_params[3:]

            if R_optimized is None:
                R_optimized = R_batch
                t_optimized = t_batch
            else:
                # Combine transformations from different batches
                R_optimized = np.dot(R_batch, R_optimized)
                t_optimized += t_batch

        return R_optimized, t_optimized

    def icp(self, T=np.eye(4)):
        initial_guess = np.zeros(6)  # Initialize with zeros
        R_optimized, t_optimized = self.optimize(initial_guess)
        return np.vstack((np.hstack((R_optimized, t_optimized.reshape(-1, 1))),
                          np.array([0, 0, 0, 1])))
