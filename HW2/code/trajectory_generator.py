import numpy as np
import matplotlib.pyplot as plt
import os
import urllib.request

def natural_cubic_spline(x, y, x_new):
    """
    Computes natural cubic spline interpolation using pure NumPy.
    """
    n = len(x)
    h = np.diff(x)
    b = np.diff(y) / h
    
    A = np.zeros((n, n))
    B = np.zeros(n)
    
    A[0, 0] = 1.0
    A[n-1, n-1] = 1.0
    
    for i in range(1, n-1):
        A[i, i-1] = h[i-1]
        A[i, i] = 2.0 * (h[i-1] + h[i])
        A[i, i+1] = h[i]
        B[i] = 6.0 * (b[i] - b[i-1])
        
    M = np.linalg.solve(A, B)
    
    # Evaluate spline
    y_new = np.zeros_like(x_new)
    
    idx = np.searchsorted(x, x_new)
    idx = np.clip(idx, 1, n-1) - 1 # Interval i is [x_i, x_{i+1}]
    
    hi = h[idx]
    dx1 = x[idx+1] - x_new
    dx2 = x_new - x[idx]
    
    term1 = (M[idx] * dx1**3) / (6.0 * hi)
    term2 = (M[idx+1] * dx2**3) / (6.0 * hi)
    term3 = (y[idx] - (M[idx] * hi**2) / 6.0) * (dx1 / hi)
    term4 = (y[idx+1] - (M[idx+1] * hi**2) / 6.0) * (dx2 / hi)
    
    y_new = term1 + term2 + term3 + term4
    return y_new

def generate_speed_profile(path_x, path_y, max_v=20.0, max_lat_acc=2.0, max_long_acc=1.0, max_long_dec=1.0):
    dx = np.diff(path_x)
    dy = np.diff(path_y)
    ds = np.sqrt(dx**2 + dy**2)
    ds = np.append(ds, ds[-1]) 
    
    xp = np.gradient(path_x) / ds
    yp = np.gradient(path_y) / ds
    xpp = np.gradient(xp) / ds
    ypp = np.gradient(yp) / ds

    curvature = np.abs(xp * ypp - yp * xpp) / np.power(xp**2 + yp**2, 1.5)
    curvature[np.isnan(curvature)] = 0.0
    curvature = np.clip(curvature, 0, 10.0)

    # TODO 3.1.b Speed limit from curvature
    # v_{max} = sqrt(a_{lat} / k)
    # cap at max_v
    v_ref = np.sqrt(max_lat_acc / (curvature + 1e-8))
    v_ref = np.minimum(max_v, v_ref)
    # [end] TODO 3.1.b
    # TODO 3.1.c Longitudinal Smoothing
    pass
    # [end] TODO 3.1.c

    return v_ref, curvature

def adaptive_sampling(px, py, curvature, v_ref=None, min_ds=3.0, max_ds=100.0, k_gain=150.0):
    """
    Adaptive Sampling Strategy:
    Larger distance step on straight paths, smaller step on curves.
    """
    dx = np.diff(px)
    dy = np.diff(py)
    ds = np.sqrt(dx**2 + dy**2)
    
    sx, sy = [px[0]], [py[0]]
    if v_ref is not None:
        sv = [v_ref[0]]
    else:
        sv = []
        
    acc_s = 0.0
    
    for i in range(1, len(px)):
        acc_s += ds[i-1]
        desired_ds = max_ds / (1.0 + k_gain * curvature[i])
        desired_ds = max(min_ds, min(max_ds, desired_ds))
        
        if acc_s >= desired_ds:
            sx.append(px[i])
            sy.append(py[i])
            if v_ref is not None:
                sv.append(v_ref[i])
            acc_s = 0.0
            
    if v_ref is not None:
        return np.array(sx), np.array(sy), np.array(sv)
    return np.array(sx), np.array(sy)

def uniform_sampling(px, py, v_ref=None, step_ds=5.0):
    """
    Uniform Sampling Strategy:
    Samples waypoints at a constant distance interval along the path.
    """
    dx = np.diff(px)
    dy = np.diff(py)
    ds_arr = np.sqrt(dx**2 + dy**2)
    
    sx, sy = [px[0]], [py[0]]
    if v_ref is not None:
        sv = [v_ref[0]]
    else:
        sv = []
        
    acc_s = 0.0
    
    for i in range(1, len(px)):
        acc_s += ds_arr[i-1]
        
        if acc_s >= step_ds:
            sx.append(px[i])
            sy.append(py[i])
            if v_ref is not None:
                sv.append(v_ref[i])
            acc_s = 0.0
            
    if v_ref is not None:
        return np.array(sx), np.array(sy), np.array(sv)
    return np.array(sx), np.array(sy)

if __name__ == '__main__':
    TRACK_NAME = "Silverstone"
    filename = f"tracks/{TRACK_NAME}.csv"
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    path_x = data[:, 0]
    path_y = data[:, 1]
    
    # Dynamic limitation
    max_v = 80.0
    max_lat_acc = 30.0
    max_long_acc = 12.0
    max_long_dec = 18.0
    v_ref, k = generate_speed_profile(
        path_x, path_y, 
        max_v=max_v,        
        max_lat_acc=max_lat_acc,   
        max_long_acc=max_long_acc,  
        max_long_dec=max_long_dec   
    )

    # Verify your speed profile
    test_filepath = f"tracks/{TRACK_NAME}_vref_test.npy"
    print(f"[{TRACK_NAME}] Verifying speed profile against test data...")
    expected_v_ref = np.load(test_filepath)
    
    if len(expected_v_ref) != len(v_ref):
        print("[FAIL] The length of the generated v_ref does not match the test data.")
    else:
        rmse = np.sqrt(np.mean((v_ref - expected_v_ref)**2))
        max_error = np.max(np.abs(v_ref - expected_v_ref))
        
        if rmse < 1e-4 and max_error < 1e-3:
            print(f"[PASS] Your speed profile matches the test data perfectly! (RMSE: {rmse:.6f}, Max Error: {max_error:.6f})")
        else:
            print(f"[FAIL] Your speed profile differs from the test data.")
            print(f"   -> Root Mean Square Error: {rmse:.6f}")
            print(f"   -> Maximum Error: {max_error:.6f}")

    sampled_x, sampled_y, sampled_v = adaptive_sampling(path_x, path_y, k, v_ref=v_ref)

    plt.figure(figsize=(12, 10))
    plt.title("Speed Profile")
    plt.plot(sampled_x, sampled_y, 'rx', label=f'Adaptive Waypoints', markersize=10) 
        
    sc = plt.scatter(path_x, path_y, c=v_ref, cmap='jet', s=5, label='Interpolated Dense Path')
    plt.colorbar(sc, label='Target Speed (m/s)')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()