"""Author: Arian Kourangi """
import numpy as np
from scipy.spatial import KDTree
import copy


class ScanMatch:
    def __init__(self, source, target, max_it, threshold_constant, tolerance, divergence, tree, bool = True):
        if not bool:
            self.tree = KDTree(target, copy_data=True)
        else:
            self.tree = tree
        self.source = source
        self.target = target
        self.max_iterations = max_it
        self.threshold_constant = threshold_constant
        self.tolerance = tolerance
        self.divergence_threshold = divergence

    def find_correspondences(self):
        distances, indices = self.tree.query(self.source)
        correspondences = self.target[indices]
        return distances, correspondences

    def compute_transform(self, source, target):
        source_centered = source - np.mean(source, axis=0)
        target_centered = target - np.mean(target, axis=0)

        H = np.dot(target_centered.T, source_centered)
        U, _, Vt = np.linalg.svd(H)
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
        T_tot = T.copy()
        prev_T = T_tot.copy()  # Store previous transformation
        divergence_count = 0  # Counter for divergence detection

        for i in range(self.max_iterations):
            if i == 0:
                """This does coarse alignment, first get them closely to each other before starting to do correspondence search."""
                self.transform_points(T)
            else:
                distances, correspondences = self.find_correspondences()
                median_distance = np.median(distances)
                mad = np.median(np.abs(distances - median_distance))
                threshold = self.threshold_constant * mad

                inliers_mask = np.abs(distances - median_distance) < threshold
                # inliers_mask = distances < 2*np.mean(distances)
                source_inliers = self.source[inliers_mask]
                correspondences_inliers = correspondences[inliers_mask]
                T_new = self.compute_transform(
                    source_inliers, correspondences_inliers)
                T_tot = np.dot(T_new, T_tot)
                self.transform_points(T_new)

                change = np.abs(T_tot[0:2, 3] - T[0:2, 3]).max()
                if change < self.divergence_threshold:
                    pass
                else:
                    print("Divergence detected at iteration", i)
                    T_tot = None  # reset to initial guess
                    break
                # Check for convergence
                if np.allclose(T_tot, prev_T, atol=self.tolerance):
                    print("Converged at iteration", i)
                    break
                prev_T = T_tot.copy()  # Update previous transformation

                # Check for divergence

        # print("Iterations:", i)
        return T_tot
