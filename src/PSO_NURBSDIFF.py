import os
import numpy as np
import open3d as o3d
import pyswarms as ps
import time
from scipy.spatial.distance import cdist, directed_hausdorff
import csv
from geomdl import operations, exchange, BSpline, utilities
import matplotlib.pyplot as plt
import torch
torch.manual_seed(120)
from tqdm import tqdm
from NURBSDiff.surf_eval import SurfEval
from torch.autograd import Variable
import copy
import os
from geomdl import exchange
from geomdl.exchange import export_smesh, import_smesh, export_stl
from collections import defaultdict
import gc
import sys
import argparse
import math
import config
import matplotlib.ticker as ticker



def process_pointcloud(point_cloud_file):
    pcd_real = o3d.io.read_point_cloud(point_cloud_file)
    # Preprocessing
    pcd_outlier_removed, _ = pcd_real.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    desired_num_points = 671
    pcd_down_real = pcd_outlier_removed.farthest_point_down_sample(desired_num_points)
    pcd_real_np = np.asarray(pcd_down_real.points)

    min_coord = np.min(pcd_real_np, axis=0)
    max_coord = np.max(pcd_real_np, axis=0)

    scale_factor = max(max_coord - min_coord)
    # Normalize the coordinates
    pcd_real_np_scaled = (pcd_real_np - min_coord) / scale_factor

    # Create a new point cloud with the normalized coordinates
    pcd_cm_real_scaled = o3d.geometry.PointCloud()
    pcd_cm_real_scaled.points = o3d.utility.Vector3dVector(pcd_real_np_scaled)
    pcd_cm_real_scaled.paint_uniform_color([0, 0, 0])  # Set all points to black

    return pcd_cm_real_scaled, scale_factor, min_coord


def convert_optim_variables(position, **kwargs):
    result_array = np.zeros((n_particles, 54), dtype=np.float64)  # Create an array of 54 zeros (n_particles, leaf_num*18*3)
    rotate_xy = np.zeros((n_particles, 1), dtype=np.float64)
    for j in range(position.shape[0]): # position(n_particle, dimention)
        
        #x
        result_array[j, 0:18:3] = position[j, 0:6]
        cumsum_x = np.cumsum(result_array[j, 0:18:3])/4.2
        for i in range(3):
            result_array[j, i*18:(i+1)*18:3] = cumsum_x

        #y
        del_y_end = position[j, 11]
        del_y = position[j, 6:11]
        result_array[j, 19:36:3] = position[j, 12:18]
        result_array[j, 16] = result_array[j, 34] - del_y_end
        result_array[j, 52] = result_array[j, 34] + del_y_end
        result_array[j, 1:16:3] = result_array[j, 19:33:3] - del_y
        result_array[j, 37:52:3] = result_array[j, 19:33:3] + del_y

        #z
        result_array[j, 2:18:3] = position[j, 18:24]
        result_array[j, 20] = result_array[j, 38]= position[j, 18]
        result_array[j, 35] = result_array[j, 53]= position[j, 23]
        result_array[j, 23:34:3] = position[j, 24:28]
        result_array[j, 41:52:3] = position[j, 28:32]

        if position.shape[1] == 33:
            # rotate_xy
            degree = position[j, 32]
            rotate_xy[j, 0] = degree
        if position.shape[1] == 32:
            rotate_xy[j, 0] = kwargs.get('rotate_xy_list', None)

    result_array = result_array.round(6)
    rotate_xy = rotate_xy.round(6)
    return result_array, rotate_xy


def create_multiple_surfaces(point_sets, rotate_xy, visualize=False, size_u=3, size_v=6, degree_u=2, degree_v=3):

    all_evaluated_points = []
    delta = 0.04
    if not visualize:
        for points, angle in zip(point_sets, rotate_xy):
            
            surface = BSpline.Surface()
            # Set degrees
            surface.degree_u = degree_u
            surface.degree_v = degree_v

            # Set control points size
            surface.ctrlpts_size_u = size_u
            surface.ctrlpts_size_v = size_v 

            # Ensure points are an array of 3D points
            if isinstance(points, np.ndarray) and points.ndim == 1 and points.size % 3 == 0:
                points = points.reshape(-1, 3)
            elif not (isinstance(points, np.ndarray) and points.shape[1] == 3):
                raise ValueError("Points must be a 2D numpy array with three columns")
            
            surface.ctrlpts2d = [points[i:i+size_v].tolist() for i in range(0, len(points), size_v)]
            
            # knot vectors
            surface.knotvector_u = utilities.generate_knot_vector(surface.degree_u, surface.ctrlpts_size_u)
            surface.knotvector_v = utilities.generate_knot_vector(surface.degree_v, surface.ctrlpts_size_v)

            # Evaluate the surface
            surface.evaluate()

            surface.delta = delta
            surface = operations.rotate(surface, angle, inplace=True)

            evalpts = np.array(surface.evalpts)
            all_evaluated_points.append(evalpts)

    if visualize:
        for points in point_sets:
            surface = BSpline.Surface()
            # Set degrees
            surface.degree_u = degree_u
            surface.degree_v = degree_v

            # Set control points size
            surface.ctrlpts_size_u = size_u
            surface.ctrlpts_size_v = size_v 
            # Ensure points are an array of 3D points
            if isinstance(points, np.ndarray) and points.ndim == 1 and points.size % 3 == 0:
                points = points.reshape(-1, 3)
            elif not (isinstance(points, np.ndarray) and points.shape[1] == 3):
                raise ValueError("Points must be a 2D numpy array with three columns")

            surface.ctrlpts2d = [points[i:i+size_v].tolist() for i in range(0, len(points), size_v)]
            
            # knot vectors
            surface.knotvector_u = utilities.generate_knot_vector(surface.degree_u, surface.ctrlpts_size_u)
            surface.knotvector_v = utilities.generate_knot_vector(surface.degree_v, surface.ctrlpts_size_v)

            # Evaluate the surface
            surface.evaluate()

            surface.delta = delta
            surface = operations.rotate(surface, rotate_xy, inplace=True)

            evalpts = np.array(surface.evalpts)
            all_evaluated_points.append(evalpts)

    return all_evaluated_points, surface


iter_num = 0
def objective_function(variables, **kwargs):
    global iter_num
    iter_num += 1
    positions, rotate_xy = convert_optim_variables(variables, **kwargs)

    mesh_eval_points, surf = create_multiple_surfaces(positions, rotate_xy)

    # Initialize lists to store the values
    loss = []
    hd_values = []
    cd_values = []
    iter = []
    particle = []

    for l in range(positions.shape[0]):
        centroid_pcd_mesh = np.mean(mesh_eval_points[l], axis=0)
        centroid_pcd_cm_real_scaled = np.mean(np.asarray(pcd_cm_real_scaled.points), axis=0)
        translation = centroid_pcd_mesh - centroid_pcd_cm_real_scaled
        # Translate pcd to align with pcd_mesh
        pcd_aligned_real = pcd_cm_real_scaled.translate(translation)

        # Convert point clouds to NumPy arrays
        pcd_real_rescaled = np.asarray(pcd_aligned_real.points)
        pcd_mesh_rescaled = mesh_eval_points[l]
        x = kwargs.get('x')

        # Calculate the pairwise distances
        distances_pcd_to_mesh = cdist(pcd_real_rescaled, pcd_mesh_rescaled)
        distances_mesh_to_pcd = cdist(pcd_mesh_rescaled, pcd_real_rescaled)
        # Chamfer distance
        chamfer_distance = np.mean(np.min(distances_pcd_to_mesh, axis=1)) + np.mean(np.min(distances_mesh_to_pcd, axis=1))

        # Hausdorff distance
        hausdorff_distance = max(directed_hausdorff(pcd_real_rescaled, pcd_mesh_rescaled)[0], directed_hausdorff(pcd_mesh_rescaled, pcd_real_rescaled)[0])

        dist = chamfer_distance + (x * hausdorff_distance)  

        loss.append(dist)
        hd_values.append(hausdorff_distance)
        cd_values.append(chamfer_distance)
        iter.append(iter_num)
        particle.append(l+1)

    loss_arr = np.array(loss)
    # Save values to CSV file
    with open(f"{config.CSV_DIR}/{plant}_L{lf}_values_{nn}.csv", 'a', newline='') as csvfile:
    
        csvwriter = csv.writer(csvfile)
        for it, part, cd, hd in zip(iter, particle, cd_values, hd_values):
            csvwriter.writerow([it, part, cd, hd])

    return loss_arr


def plot_and_save(history, title, xlabel, ylabel, file_path):
    plt.figure(figsize=(6, 6))
    plt.plot(history)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.savefig(file_path, dpi=300, bbox_inches='tight')
    plt.close()


def optimize_PSO():
    start_time = time.time()
    opts = {'c1':2.5, 'c2':0.5, 'w':0.9}
    oh_strategy={ "w":'exp_decay', "c1":'lin_variation',"c2":'lin_variation'}
    optim_kwargs_dict = {}

    if dimensions == 33:

        bounds_group1 = ([0.2] * 6, [1.2] * 6)  # x
        bounds_group20 = ([0.0] * 1, [0.1] * 1)  # delta_y1
        bounds_group21 = ([0.05] * 4, [0.2] * 4)  # delta_y2, 3, 4, 5
        bounds_group22 = ([0.0] * 1, [0.1] * 1)  # delta_y6
        bounds_group23 = ([-0.1] * 6, [0.1] * 6)  # y
        bounds_group3 = ([0.0] * 14, [1.0] * 14)  #z
        bounds_group4 = ([0.0] * 1, [360.0] * 1)  #rotate_xy

        # Combine all bounds into a single list
        bounds = bounds_group1[0] + bounds_group20[0] + bounds_group21[0] + bounds_group22[0] + bounds_group23[0] + bounds_group3[0] + bounds_group4[0], \
                bounds_group1[1] + bounds_group20[1] + bounds_group21[1] + bounds_group22[1] + bounds_group23[1] + bounds_group3[1] + bounds_group4[1]

        # Create an instance of the PSO optimizer
        optimizer2 = ps.single.GlobalBestPSO(n_particles=n_particles, dimensions=dimensions, options=opts, bounds=bounds, oh_strategy=oh_strategy)
        
        optim_kwargs_dict['x'] = 0.1
        # Run the optimization
        best_cost, best_pos = optimizer2.optimize(objective_function, iters=iters, verbose=True, **optim_kwargs_dict)
        cost_history = optimizer2.cost_history
        pos_history = optimizer2.pos_history
        best_pos_history =optimizer2.best_pos_history

        end_time1 = time.time()
        execution_time = end_time1 - start_time

        data2 = {'best_cost':best_cost, 'best_pos':best_pos, 'cost_history': cost_history, 'best_pos_history' : best_pos_history, 'pos_history' : pos_history, 'Execution Time' : round(execution_time/60, 2)}

        # Ensure the directory exists
        npy_dir_PSO = config.NPY_PSO_DIR
        if not os.path.exists(npy_dir_PSO):
            os.makedirs(npy_dir_PSO)

        # Define the full path to the file
        file_path = os.path.join(npy_dir_PSO, f'{plant}_L{lf}_results_{nn}.npy')

        # Save the numpy file
        np.save(file_path, data2)


    if dimensions == 32:
        bounds_group1 = ([0.2] * 6, [1.2] * 6)  # x
        bounds_group20 = ([0.0] * 1, [0.1] * 1)  # delta_y1
        bounds_group21 = ([0.05] * 4, [0.2] * 4)  # delta_y2, 3, 4, 5
        bounds_group22 = ([0.0] * 1, [0.1] * 1)  # delta_y6
        bounds_group23 = ([-0.1] * 6, [0.1] * 6)  # y
        bounds_group3 = ([0.0] * 14, [1.0] * 14)  #z
        
        # Combine all bounds into a single list
        bounds = bounds_group1[0] + bounds_group20[0] + bounds_group21[0] + bounds_group22[0] + bounds_group23[0] + bounds_group3[0], \
                bounds_group1[1] + bounds_group20[1] + bounds_group21[1] + bounds_group22[1] + bounds_group23[1] + bounds_group3[1]

        data2 = np.load(f'{config.NPY_PSO_DIR}/{plant}_L{lf}_results_{nn-1}.npy', allow_pickle=True).item()

        best_pos2 = data2['best_pos']
        # Create an instance of the PSO optimizer
        optimizer = ps.single.GlobalBestPSO(n_particles=n_particles, dimensions=dimensions, options=opts, bounds=bounds, oh_strategy=oh_strategy)

        optim_kwargs_dict['x'] = 0.1
        optim_kwargs_dict['rotate_xy_list'] = best_pos2[32]
        # Run the optimization
        best_cost, best_pos = optimizer.optimize(objective_function, iters=iters, verbose=True, **optim_kwargs_dict)
        cost_history = optimizer.cost_history
        pos_history = optimizer.pos_history
        best_pos_history =optimizer.best_pos_history

        plot_file_path = config.COST_PLOT_DIR
        if not os.path.exists(plot_file_path):
            os.makedirs(plot_file_path)
        plt_file_path = os.path.join(plot_file_path, f'{plant}_L{lf}_cost_{nn}.png')
        plot_and_save(cost_history, "Cost History (CD+0.1HD)", "Iteration", "Cost", plt_file_path)
        

        end_time1 = time.time()
        execution_time = end_time1 - start_time

        data = {'best_cost':best_cost, 'best_pos':best_pos, 'cost_history': cost_history, 'best_pos_history' : best_pos_history, 'pos_history' : pos_history, 'Execution Time' : round(execution_time/60, 2)}
        np.save(f'{config.NPY_PSO_DIR}/{plant}_L{lf}_results_{nn}.npy', data)

    end_time = time.time()
    execution_time = end_time - start_time
    print("Execution Time:", round(execution_time/60, 2), "minutes")

    return best_cost, best_pos


def chamfer_distance_one_side(pred, gt, side=1):
    """
    Computes average chamfer distance prediction and groundtruth
    but is one sided
    :param pred: Prediction: B x N x 3
    :param gt: ground truth: B x M x 3
    :return:
    """
    # Ensure both tensors are on the same device
    device = pred.device 
    gt = gt.to(device) 

    if isinstance(pred, np.ndarray):
        pred = Variable(torch.from_numpy(pred.astype(np.float32))).cuda()

    if isinstance(gt, np.ndarray):
        gt = Variable(torch.from_numpy(gt.astype(np.float32))).cuda()

    pred = torch.unsqueeze(pred, 1)
    gt = torch.unsqueeze(gt, 2)

    diff = pred - gt
    diff = torch.sum(diff ** 2, 3)
    if side == 0:
        cd = torch.mean(torch.min(diff, 1)[0], 1)
    elif side == 1:
        cd = torch.mean(torch.min(diff, 2)[0], 1)
    cd = torch.mean(cd)
    return cd


def downsample_points_farthest(points, num_samples):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    # pcd_downsampled = pcd.farthest_point_down_sample(num_samples)
    pcd_downsampled = pcd.uniform_down_sample(every_k_points=10)
    return np.asarray(pcd_downsampled.points)


def main():

    chamfer_data = {} 
    max_retries = 3 
    def optimize_leaf():
        opt.zero_grad()

        out = layer(torch.cat((inpCtrlPts.unsqueeze(0), weight), axis=-1))

        # Calculate second-order finite differences
        der11 = (2*out[:, 1:-1, 1:-1, :] - out[:, 0:-2, 1:-1, :] - out[:, 2:, 1:-1, :]) ** 2
        # der22 = (2*out[:, 1:-1, 1:-1, :] - out[:, 1:-1, 0:-2, :] - out[:, 1:-1, 2:, :]) ** 2
        der12 = (2*out[:, 1:-1, 1:-1, :] - out[:, 0:-2, 1:-1, :] - out[:, 1:-1, 2:, :]) ** 2
        # der21 = (2*out[:, 1:-1, 1:-1, :] - out[:, 1:-1, 0:-2, :] - out[:, 2:, 1:-1, :]) ** 2

        # Sum the squared differences for curvature penalty
        curvature_penalty1 =  torch.sum(torch.abs(der11))
        curvature_penalty2 = torch.sum(torch.abs(der12))

        lossVal = 0
        if device.type == 'cuda':
            lossVal = chamfer_distance_one_side(out.view(1, uEvalPtSize * vEvalPtSize, 3), target.view(1, mumPoints[0], 3).cuda())
        else:
            lossVal = chamfer_distance_one_side(out.view(1, uEvalPtSize * vEvalPtSize, 3), target.view(1, mumPoints[0], 3))

        lossVal += 0.01 * torch.abs(curvature_penalty1)
        lossVal += 0.0001 * torch.abs(curvature_penalty2)
        
        # Add constraint to align the first and last three control points in the v-direction
        proximity_penalty = 0
        for v in range(0, CtrlPtsCountUV[1], 5):
            p1 = inpCtrlPts[0, v, :]
            p2 = inpCtrlPts[1, v, :]
            p3 = inpCtrlPts[2, v, :]
            proximity_penalty += torch.sum((p1 - p2) ** 2) + torch.sum((p1 - p3) ** 2) + torch.sum((p2 - p3) ** 2)

        lossVal += 0.0008 * torch.abs(proximity_penalty)  

        # Back propagate
        lossVal.backward(retain_graph=True)
        return lossVal
    
    # Start the timer for NURBS-Diff optimization
    start_time_nurbsdiff = time.time()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    uEvalPtSize = 8
    vEvalPtSize = 32
    print(uEvalPtSize, "  ",vEvalPtSize)

    # num_samples = 128

    for l in range(1, leaf_num+1):
        retries = 0
        success = False
        while retries < max_retries and not success:
            try:
                dataFileName = f'{config.PLANT_DIR_DAT}/{plant}_L{l}.dat'
                smeshInFileName = f"{config.OUT_PSO_DIR}/{plant}_L{l}_PSO{iters}_{nn}.dat"
                dir_pth = f"{config.OUT_NURBS_DIR}"
                if not os.path.exists(dir_pth):
                    os.makedirs(dir_pth)
                smeshOutFileName = f"{config.OUT_NURBS_DIR}/{plant}_L{l}_nurbs{uEvalPtSize}{vEvalPtSize}.dat"
                OutFileName_stl = f"{config.OUT_NURBS_DIR}/{plant}_L{l}_nurbs{uEvalPtSize}{vEvalPtSize}.stl"
                
                surface = import_smesh(smeshInFileName)[0]

                degree = [surface.degree_u, surface.degree_v]
                CtrlPtsCountUV = [surface.ctrlpts_size_u, surface.ctrlpts_size_v]
                knotU = surface.knotvector_u
                knotV = surface.knotvector_v

                CtrlPts = np.array(surface.ctrlptsw)

                CtrlPtsNoW = np.reshape(CtrlPts[:, :3], [CtrlPtsCountUV[0], CtrlPtsCountUV[1], 3])
        
                target = torch.from_numpy(np.genfromtxt(dataFileName, delimiter=' ', dtype=np.float32))

                
                # # Downsample target points using farthest point sampling
                # target_np = target.cpu().numpy()
                # target_downsampled_np = downsample_points_farthest(target_np, num_samples)
                # target = torch.from_numpy(target_downsampled_np).float()

                mumPoints = target.cpu().shape

                layer = SurfEval(CtrlPtsCountUV[0], CtrlPtsCountUV[1], knot_u=knotU, knot_v=knotV, dimension=3,
                                p=degree[0], q=degree[1], out_dim_u=uEvalPtSize, out_dim_v=vEvalPtSize, dvc=device.type)
            
                if device.type == 'cuda':
                    inpCtrlPts = torch.nn.Parameter(torch.from_numpy(copy.deepcopy(CtrlPtsNoW)).cuda())
                    inpWeight = torch.ones(1, CtrlPtsCountUV[0], CtrlPtsCountUV[1], 1).cuda()
        
                else:
                    inpCtrlPts = torch.nn.Parameter(torch.from_numpy(copy.deepcopy(CtrlPtsNoW)))
                    inpWeight = torch.ones(1, CtrlPtsCountUV[0], CtrlPtsCountUV[1], 1)


                # Calculate Chamfer distance before optimization
                pred_points_before = layer(torch.cat((inpCtrlPts.unsqueeze(0), inpWeight), axis=-1))
                chamfer_dist_before = chamfer_distance_one_side(pred_points_before.view(1, uEvalPtSize * vEvalPtSize, 3), target.view(1, mumPoints[0], 3))
                print(f"Leaf{l} Initial Chamfer Distance : {chamfer_dist_before.item()}")

                # Store the before optimization Chamfer distance
                chamfer_data[f'Leaf{l}'] = {'before': chamfer_dist_before.item(), 'after': None}


                opt = torch.optim.Adam(iter([inpCtrlPts]), lr=1e-2) #1e-3
                scheduler = torch.optim.lr_scheduler.MultiStepLR(opt, milestones=[50,100,150,200,250,300], gamma=0.5) #500
                pbar = tqdm(range(500))

                if device.type == 'cuda':
                    weight = torch.ones(1, CtrlPtsCountUV[0], CtrlPtsCountUV[1], 1).cuda()
                else:
                    weight = torch.ones(1, CtrlPtsCountUV[0], CtrlPtsCountUV[1], 1)

                for i in pbar:
                    # Optimize step
                    lossVal = opt.step(optimize_leaf)
                    scheduler.step()
                    out = layer(torch.cat((inpCtrlPts.unsqueeze(0), weight), axis=-1))

                    if math.isnan(lossVal.item()):
                        raise ValueError(f"NaN detected in loss for Leaf {l} at step {i}")

                    pbar.set_description("{}_L{} Total loss is (%s): %s".format(plant, l) % (i + 1, lossVal.item()))
                    pass


                # Calculate Chamfer distance after NURBS-Diff
                pred_points_after = layer(torch.cat((inpCtrlPts.unsqueeze(0), weight), axis=-1))
                chamfer_dist_after = chamfer_distance_one_side(pred_points_after.view(1, uEvalPtSize * vEvalPtSize, 3), target.view(1, mumPoints[0], 3))
                print(f"Leaf{l} Final Chamfer Distance: {chamfer_dist_after.item()}")

                if math.isnan(chamfer_dist_after.item()):
                    raise ValueError(f"NaN detected in Chamfer distance for Leaf {l} after optimization")

                # Store the after optimization Chamfer distance
                chamfer_data[f'Leaf{l}']['after'] = chamfer_dist_after.item()

                # Save Chamfer and other useful data
                npy_dir_nurbs = config.NPY_NURBS_DIFF_DIR
                
                if not os.path.exists(npy_dir_nurbs):
                    os.makedirs(npy_dir_nurbs)

                predCtrlPts = torch.cat((inpCtrlPts.unsqueeze(0), weight), axis=-1).detach().cpu().numpy().squeeze()
                surface.ctrlptsw = (np.reshape(predCtrlPts,(CtrlPtsCountUV[0]*CtrlPtsCountUV[1], 4)).tolist())
                export_smesh(surface, smeshOutFileName)
                export_stl(surface, OutFileName_stl)

                # End the timer for NURBS-Diff optimization
                end_time_nurbsdiff = time.time()
                nurbsdiff_time = end_time_nurbsdiff - start_time_nurbsdiff  # Time in seconds

                # Store results in a dictionary and save as an npy file
                results_dict = {
                    'initial_chamfer_distance': chamfer_dist_before.item(),
                    'final_chamfer_distance': chamfer_dist_after.item(),
                    'initial_control_points': CtrlPtsNoW,  # Control points before NURBS-Diff optimization
                    'optimized_control_points': inpCtrlPts.detach().cpu().numpy(), # Control points after optimization
                    'final_surface_points': pred_points_after.detach().cpu().numpy(),  # Final evaluated surface points after optimization
                    'loss_history': lossVal.item(),  # store loss history
                    'NURBS_Diff_time_seconds': nurbsdiff_time
                }

                np.save(f"{npy_dir_nurbs}/{plant}_Leaf{l}_NURBS_Diff_Results_{uEvalPtSize}{vEvalPtSize}.npy", results_dict)

                del inpCtrlPts, inpWeight, layer, target, CtrlPtsNoW, surface
                gc.collect()

                success = True  # Successfully completed without NaN

            except ValueError as e:
                retries += 1
                print(f"Error encountered for Leaf {l}: {e}. Retrying... ({retries}/{max_retries})")

            except Exception as e:
                print(f"Unexpected error for Leaf {l}: {e}")
                break
        if not success:
            print(f"Failed to process Leaf {l} after {max_retries} attempts. Moving to the next leaf.")


    # Plot Chamfer distance convergence
    plot_chamfer_convergence(chamfer_data, plant, uEvalPtSize, vEvalPtSize)

    gc.collect()


def plot_chamfer_convergence(chamfer_data, plant, uEvalPtSize, vEvalPtSize):
    """ 
    Plot the reduction in Chamfer distance (in mm) before and after NURBS_Diff for each leaf 
    using a bar plot to show the improvement clearly, with colorblind-friendly colors.
    """
    # Extract data for plotting
    leaves = list(chamfer_data.keys())
    before = [chamfer_data[leaf]['before'] * 1000 for leaf in leaves]  # Convert from meters to mm
    after = [chamfer_data[leaf]['after'] * 1000 for leaf in leaves]    # Convert from meters to mm

    x = np.arange(len(leaves))  # the label locations
    width = 0.35  # the width of the bars

    colors = ["#88CCEE", "#CC6677"]  # Blue, Red

    # Create the plot
    fig, ax = plt.subplots(figsize=(12, 6))
    bars1 = ax.bar(x - width/2, before, width, label='After PSO, Before NURBS_Diff (mm)', color=colors[0])
    bars2 = ax.bar(x + width/2, after, width, label='After NURBS_Diff (mm)', color=colors[1])

    # Add labels, title, and tick customization
    ax.set_xlabel('Leaf Index', fontsize=14)
    ax.set_ylabel('Chamfer Distance (mm)', fontsize=14)  # Update unit to mm
    ax.set_title(f'Chamfer Distance Reduction (mm) Before and After NURBS-Diff for {plant}', fontsize=16, pad=20)
    ax.set_xticks(x)
    ax.set_xticklabels(leaves, rotation=45, fontsize=12)
    ax.legend(fontsize=12)

    # Adding gridlines and formatting the y-axis for cleaner display
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.tick_params(axis='y', labelsize=12)
    ax.tick_params(axis='x', labelsize=12)

    # Adding text annotations to show the exact value on top of the bars
    for bar in bars1 + bars2:
        yval = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.2f}', ha='center', va='bottom', fontsize=10)

    # Adjust layout and save the plot
    plt.tight_layout()
    if not os.path.exists(config.CHAMFER_CONVERGENCE_DIR):
        os.makedirs(config.CHAMFER_CONVERGENCE_DIR)
    plt.savefig(f"{config.CHAMFER_CONVERGENCE_DIR}/{plant}_Chamfer_Convergence_mm_{uEvalPtSize}{vEvalPtSize}.png", dpi=300)
    plt.show()


def save_execution_time(log_file, section, execution_time):
    """
    Save the execution time for a specific section to a log file.
    
    Args:
        log_file (str): Path to the log file.
        section (str): Section name (e.g., 'PSO', 'NURBS-Diff').
        execution_time (float): Execution time in seconds.
    """
    with open(log_file, 'a') as file:
        file.write(f"{section}: {execution_time:.2f} seconds\n")


if __name__ == "__main__":
    start_time = time.time()
    np.random.seed(42)

    parser = argparse.ArgumentParser(description="Process plant data.")
    parser.add_argument('--plant', type=str, required=True, help='Plant name')
    parser.add_argument('--leaf_count', type=int, required=True, help='Number of leaves for the plant')

    args = parser.parse_args()

    plant = args.plant
    leaf_num = args.leaf_count

    print(f"Processing plant: {plant} with {leaf_num} leaves")

    n_particles = 300
    n_prtcl = n_particles

    for lf in range (1, leaf_num+1):
        nn = 1
        iters = 1
        dimensions = 33
        point_cloud_file_leaf = f'{config.PLANT_DIR_PLY}/{plant}_L{lf}.ply'
        
        csv_dir = config.CSV_DIR
        if not os.path.exists(config.CSV_DIR):
            os.makedirs(config.CSV_DIR)
        csv_file_path = f"{csv_dir}/{plant}_L{lf}_values_{nn}.csv"
        
        with open(csv_file_path, 'w', newline='') as csvfile:
            header = ['iteration', 'Particle', 'CD', 'HD']
            writer = csv.DictWriter(csvfile, fieldnames=header)
            writer.writeheader()
        pcd_cm_real_scaled, scale_factor, min_coord = process_pointcloud(point_cloud_file_leaf)
        optimize_PSO()

        nn = 2
        dimensions = 32
        iters = 50
    
        csv_file_path = f'{csv_dir}/{plant}_L{lf}_values_{nn}.csv'
        # Ensure the directory exists
        if not os.path.exists(csv_dir):
            os.makedirs(csv_dir)
        with open(csv_file_path, 'w', newline='') as csvfile:
            header = ['iteration', 'Particle', 'CD', 'HD']
            writer = csv.DictWriter(csvfile, fieldnames=header)
            writer.writeheader()
        
        pcd_cm_real_scaled, scale_factor, min_coord = process_pointcloud(point_cloud_file_leaf)
        optimize_PSO()

    iters = 50
    n_particles= 1
    nn = 2
    for lf in range (1, leaf_num+1):
        data = np.load(f'{config.NPY_PSO_DIR}/{plant}_L{lf}_results_{nn}.npy', allow_pickle=True).item()
        print(f'{plant}_L{lf}_best_cost: ', data['best_cost'])
        cost_history = data['cost_history']
        best_pos = data['best_pos']
        print(f'{plant}_L{lf}_Execution_Time: ', data['Execution Time'])
        point_cloud_file = f'{config.PLANT_DIR_PLY}/{plant}_L{lf}.ply'

        pcd_cm_real_scaled, scale_factor, min_coord = process_pointcloud(point_cloud_file)
        
        best_pos = best_pos.reshape(1, -1)
        positions, rotate_xy = convert_optim_variables(best_pos)

        data2 = np.load(f'{config.NPY_PSO_DIR}/{plant}_L{lf}_results_{nn-1}.npy', allow_pickle=True).item()
        best_pos2 = data2['best_pos']
        rotate_xy = best_pos2[32]
        
        mesh_eval_points, surf = create_multiple_surfaces(positions, rotate_xy, visualize=True)

        # Compute translation vector
        centroid_pcd_mesh = np.mean(mesh_eval_points[0], axis=0)
        centroid_pcd_cm_real_scaled = np.mean(np.asarray(pcd_cm_real_scaled.points), axis=0)
        translation = centroid_pcd_mesh - centroid_pcd_cm_real_scaled

        # Translate pcd to align with pcd_mesh
        pcd_aligned_real = pcd_cm_real_scaled.translate(translation)
        # Convert point clouds to NumPy arrays
        pcd_real_rescaled = np.asarray(pcd_aligned_real.points)
        pcd_mesh_rescaled = mesh_eval_points[0]
        

        # Calculate the pairwise distances
        distances_pcd_to_mesh = cdist(pcd_real_rescaled, pcd_mesh_rescaled)
        distances_mesh_to_pcd = cdist(pcd_mesh_rescaled, pcd_real_rescaled)
        # Calculate the Chamfer distance
        chamfer_distance = np.mean(np.min(distances_pcd_to_mesh, axis=1)) + np.mean(np.min(distances_mesh_to_pcd, axis=1))

        # Hausdorff distance
        hausdorff_distance = max(directed_hausdorff(pcd_real_rescaled, pcd_mesh_rescaled)[0], directed_hausdorff(pcd_mesh_rescaled, pcd_real_rescaled)[0])
        
        pcd_mesh = o3d.geometry.PointCloud()
        pcd_mesh.points = o3d.utility.Vector3dVector(pcd_mesh_rescaled)

        # # Visualize the aligned point clouds
        # o3d.visualization.draw_geometries([pcd_aligned_real, pcd_mesh])


        print(f"CD_Leaf{lf} : ", round(chamfer_distance, 4))
        print(f"HD_Leaf{lf} : ", round(hausdorff_distance, 4))
        print(f"Ch_dist + 0.1 * H_dist_Leaf{lf} : ", round(chamfer_distance + (0.1 * hausdorff_distance), 4) )
        print(f"____________________End of Leaf{lf}_________________________")
        
        min_coord2 = np.min(pcd_real_rescaled, axis=0)
        pcd_real_rescale = (np.asarray(pcd_real_rescaled)-min_coord2) * scale_factor + min_coord
        pcd_mesh_rescale = (np.asarray(pcd_mesh_rescaled)-min_coord2) * scale_factor + min_coord
        translation_vector = np.array(-(min_coord2*scale_factor)+min_coord)

        operations.scale(surf, scale_factor, inplace=True)
        operations.translate(surf, (translation_vector[0], translation_vector[1], translation_vector[2]), inplace=True)
        
        dir_path = f'{config.OUT_PSO_DIR}'
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        file_name = f"{config.OUT_PSO_DIR}/{plant}_L{lf}_PSO{iters}_{nn}.dat"
        exchange.export_smesh(surf, file_name)
        
        file_name1 = f"{config.OUT_PSO_DIR}/{plant}_L{lf}_PSO{iters}_{nn}.stl"
        exchange.export_stl(surf, file_name1)
    
    end_time_pso = time.time()
    execution_time_pso = end_time_pso - start_time

    # Log PSO execution time
    log_file = f"{config.PSO_NURBSDIFF_LOG_DIR}/{plant}_execution_time.log"
    if not os.path.exists(config.PSO_NURBSDIFF_LOG_DIR):
        os.makedirs(config.PSO_NURBSDIFF_LOG_DIR)
    save_execution_time(log_file, "PSO Optimization", execution_time_pso)

    start_time_nurbsdiff = time.time()
    
    main()

    end_time_nurbsdiff = time.time()
    nurbsdiff_time = end_time_nurbsdiff - start_time_nurbsdiff

    # Log NURBS-Diff execution time
    save_execution_time(log_file, "NURBS-Diff Optimization", nurbsdiff_time)
