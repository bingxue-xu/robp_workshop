import numpy as np
from scipy.spatial import KDTree
import copy
from sklearn.linear_model import RANSACRegressor


class ScanMatch:
    def __init__(self, source, target, max_it, threshold, tolerance):
        self.tree = KDTree(target, copy_data=True)
        self.source = source
        self.target = target
        self.max_iterations = max_it
        self.distance_threshold = threshold
        self.tolerance = tolerance
        self.original_source = copy.deepcopy(source)

    def reset_algorithm(self):
        # Reset source to its original state
        self.source = self.original_source

    def coarse_alignment(self):
        # Perform a coarse alignment or registration before running ICP again
        # Here you can use any method suitable for your application,
        # like a simpler registration algorithm or manual initialization
        # For demonstration, let's simply use the identity transformation
        return np.eye(4)

    def find_correspondences_ransac(self, sample_size=3, max_trials=100):
        sample_size = min(10, len(self.source))
        if len(self.source) < sample_size:
            return None, None  # Not enough samples for RANSAC

        inliers_mask = np.zeros(len(self.source), dtype=bool)
        best_inliers_mask = None
        best_num_inliers = 0

        for _ in range(max_trials):
            # Randomly sample points from the source and target
            sample_indices = np.random.choice(
                len(self.source), size=sample_size, replace=False)
            sample_source = self.source[sample_indices]
            sample_target = self.target[sample_indices]

            # Fit a transformation model using RANSAC
            model = RANSACRegressor()
            model.fit(sample_source, sample_target)

            # Predict correspondences for all points in source
            predicted_target = model.predict(self.source)

            # Calculate residuals (distances between predicted and actual target)
            residuals = np.abs(predicted_target - self.target)

            # Identify inliers based on residuals and distance threshold
            current_inliers_mask = np.linalg.norm(
                residuals, axis=1) < self.distance_threshold
            current_num_inliers = np.sum(current_inliers_mask)

            # Update best inliers
            if current_num_inliers > best_num_inliers:
                best_num_inliers = current_num_inliers
                best_inliers_mask = current_inliers_mask.copy()

        if best_inliers_mask is None:
            # No inliers found
            return None, None

        # Filter source and target points to keep only inliers
        source_inliers = self.source[best_inliers_mask]
        target_inliers = self.target[best_inliers_mask]

        return source_inliers, target_inliers

    def compute_transform(self, source, target):
        source_centered = source - np.mean(source, axis=0)
        target_centered = target - np.mean(target, axis=0)

        H = np.dot(target_centered.T, source_centered)
        U, _, Vt = np.linalg.svd(H)
        Vt[-1, :] *= np.sign(np.linalg.det(np.dot(U, Vt)))
        R = np.dot(U, Vt)
        t = np.mean(target, axis=0).T - np.dot(R, np.mean(source, axis=0).T)
        T_new = np.eye(4)
        T_new[:3, :3] = R
        T_new[:3, 3] = t
        return T_new

    def transform_points(self, T):
        hom = np.ones(len(self.source))
        source_temp = np.column_stack((self.source, hom))
        source_r = np.dot(T, source_temp.T)
        self.source = np.transpose(source_r[0:3])

    def icp(self, T=np.eye(4)):
        T_tot = T
        prev_T = T_tot  # Store previous transformation

        for i in range(self.max_iterations):
            if i == 0:
                self.transform_points(T)
                (old, null) = np.shape(self.source)
                (new, null) = np.shape(self.target)
                size = min(old, new)
                T_new = self.compute_transform(
                    self.source[0:size, :], self.target[0:size, :])
                T_tot = np.dot(T_new, T_tot)
                self.transform_points(T_new)
            else:
                source_inliers, correspondences_inliers = self.find_correspondences_ransac()
                T_new = self.compute_transform(
                    source_inliers, correspondences_inliers)
                T_tot = np.dot(T_new, T_tot)
                self.transform_points(T_new)

                # Check for convergence
                if np.allclose(T_tot, prev_T, atol=self.tolerance):
                    print("Converged at iteration", i)
                    break
                prev_T = T_tot.copy()  # Update previous transformation

                # Handle divergence
                if i > 0 and np.allclose(T_tot, np.eye(4), atol=self.tolerance):
                    print("Diverged! Resetting and performing coarse alignment.")
                    self.reset_algorithm()
                    T_tot = self.coarse_alignment()
                    prev_T = T_tot  # Update previous transformation

        print("Iterations:", i)
        return T_tot
