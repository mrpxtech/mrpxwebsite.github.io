import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob

### hyperparameters ###
margin = 50  # margin given to sliding window width, tuned based on FOV
minpix = 50  # increased minpix for reduction of false positives
width_threshold = 50  # required width of the lane, else keep the previous fit
slope_threshold = 150  # maximum slope difference between left and right lines, else revert
curve_threshold = 20  # maximum curvature difference between left and right lines, else revert
ym_per_pix = (0.15/308)  # may need to change this based on bird's eye map
xm_per_pix = (0.15/410)  # need to tune (used for curve_threshold calculation)

def compare_images(image1, image2, image1_exp="Image 1", image2_exp="Image 2"):
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
    f.tight_layout()
    ax1.imshow(image1)
    ax1.set_title(image1_exp, fontsize=50)
    ax2.imshow(image2)
    ax2.set_title(image2_exp, fontsize=50)
    plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)

def crop_image(image):  # 410 x 308 image
    y = 210
    h = 308-y  # cover to the bottom
    x = 10
    w = 400
    img_crop = image[y:y+h, x:x+w]
    # plt.imshow(crop_img)  # for testing crop window sizes
    # plt.pause(0.0001)

    return img_crop

def abs_sobel_thresh(image, orient='x', sobel_kernel=3, thresh=(0, 255)):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    isX = True if orient == 'x' else False
    sobel = cv2.Sobel(gray, cv2.CV_64F, isX, not isX)
    abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    grad_binary = np.zeros_like(scaled_sobel)
    grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

    return grad_binary

def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobel = np.sqrt(sobelx**2 + sobely**2)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    mag_binary = np.zeros_like(scaled_sobel)
    mag_binary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1

    return mag_binary

def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi/2)):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    grad_dir = np.arctan2(abs_sobely, abs_sobelx)
    dir_binary = np.zeros_like(grad_dir)
    dir_binary[(grad_dir >= thresh[0]) & (grad_dir <= thresh[1])] = 1

    return dir_binary

def apply_thresholds(image, ksize=3):
    gradx = abs_sobel_thresh(image, orient='x', sobel_kernel=ksize, thresh=(50, 255))
    grady = abs_sobel_thresh(image, orient='y', sobel_kernel=ksize, thresh=(50, 255))
    mag_binary = mag_thresh(image, sobel_kernel=ksize, mag_thresh=(50, 255))
    dir_binary = dir_threshold(image, sobel_kernel=ksize, thresh=(0.7, 1.3))

    combined = np.zeros_like(dir_binary)
    combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1

    return combined

def bin_it(image, threshold):
    output_bin = np.zeros_like(image)
    output_bin[(image >= threshold[0]) & (image <= threshold[1])]=1
    return output_bin

def apply_color_threshold(image):
    hls_img = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
    # Compute a binary thresholded image where yellow is isolated from HLS components
    img_hls_yellow_bin = np.zeros_like(hls_img[:,:,0])
    img_hls_yellow_bin[((hls_img[:,:,0] >= 15) & (hls_img[:,:,0] <= 35)) \
                 & ((hls_img[:,:,1] >= 30) & (hls_img[:,:,1] <= 204)) \
                 & ((hls_img[:,:,2] >= 70) & (hls_img[:,:,2] <= 255)) \
                ] = 1

    # Compute a binary thresholded image where white is isolated from HLS components
    img_hls_white_bin = np.zeros_like(hls_img[:,:,0])
    img_hls_white_bin[((hls_img[:,:,0] >= 0) & (hls_img[:,:,0] <= 255)) \
                 & ((hls_img[:,:,1] >= 130) & (hls_img[:,:,1] <= 255)) \
                 & ((hls_img[:,:,2] >= 0) & (hls_img[:,:,2] <= 255)) \
                ] = 1

    # Now combine both
    img_hls_white_yellow_bin = np.zeros_like(hls_img[:,:,0])
    # img_hls_white_yellow_bin[(img_hls_yellow_bin == 1)] = 1 # only yellow bin
    img_hls_white_yellow_bin[(img_hls_yellow_bin == 1) | (img_hls_white_bin == 1)] = 1  # both white and yellow bin

    ### TODO:  Make sure all white marks to the left of the yellow lane marker is all zero mask OR specify driving on wrong side of road ###

    return img_hls_white_yellow_bin

def combine_threshold(s_binary, combined):
    combined_binary = np.zeros_like(combined)
    combined_binary[(s_binary == 1) | (combined == 1)] = 1

    return combined_binary

def warp(img):  # manual tuning
    img_size = (img.shape[1], img.shape[0])

    # for 410x308 image, corrected projection with cropped image
    src = np.float32(
        [[265, 20],
          [345, 90],
          [70, 90],
          [140, 20]])

    dst = np.float32(
        [[298, 0],
          [298, 55],
          [93, 55],
          [93, 0]])

    # for 410x308 image, corrected projection with uncropped image
    # src = np.float32(
    #     [[230, 200],
    #     [350, 300],
    #     [80, 300],
    #     [185, 200]])
    #
    # dst = np.float32(
    #     [[315, 0],
    #     [315, 55],
    #     [105, 55],
    #     [105, 0]])

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    binary_warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)

    return binary_warped, Minv

def compare_plotted_images(image1, image2, image1_exp="Image 1", image2_exp="Image 2"):
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
    f.tight_layout()

    # cropped
    ax1.imshow(image1)
    ax1.plot([360, 385], [20, 90], color='r', linewidth="5")  # x1, x2, y1, y2
    ax1.plot([385, 25], [90, 90], color='r', linewidth="5")
    ax1.plot([25, 50], [90, 20], color='r', linewidth="5")
    ax1.plot([50, 360], [20, 20], color='r', linewidth="5")
    ax1.set_title(image1_exp, fontsize=50)
    ax2.imshow(image2)
    ax2.plot([325, 325], [0, 55], color='r', linewidth="5")  # right vertical
    ax2.plot([325, 50], [55, 55], color='r', linewidth="5")  # bottom horizontal
    ax2.plot([50, 50], [55, 0], color='r', linewidth="5")  # left vertical
    ax2.plot([50, 325], [0, 0], color='r', linewidth="5")  # top horizontal
    ax2.set_title(image2_exp, fontsize=50)

    # uncropped
    # ax1.imshow(image1)
    # ax1.plot([230, 350], [200, 300], color='r', linewidth="5")  # x1, x2, y1, y2
    # ax1.plot([350, 80], [300, 300], color='r', linewidth="5")
    # ax1.plot([80, 185], [300, 200], color='r', linewidth="5")
    # ax1.plot([185, 230], [200, 200], color='r', linewidth="5")
    # ax1.set_title(image1_exp, fontsize=50)
    # ax2.imshow(image2)
    # ax2.plot([315, 315], [0, 55], color='r', linewidth="5")  # right vertical
    # ax2.plot([315, 105], [55, 55], color='r', linewidth="5")  # bottom horizontal
    # ax2.plot([105, 105], [55, 0], color='r', linewidth="5")  # left vertical
    # ax2.plot([105, 315], [0, 0], color='r', linewidth="5")  # top horizontal
    # ax2.set_title(image2_exp, fontsize=50)

    plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)


def get_histogram(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)

    return histogram

def slide_window(binary_warped, histogram):
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    midpoint = np.int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 5  # lowered number of windows based on FOV
    window_height = np.int(binary_warped.shape[0]/nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base

    # ### hyperparameters ###
    # margin = 50  # tuned based on FOV
    # minpix = 50  # increased minpix for reduction of false positives
    # ###

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
        (0,255,0), 2)
        cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
        (0,255,0), 2)
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # left_fit, left_fit_res, _, _, _ = np.polyfit(lefty, leftx, 2, full=True)  # original implementation
    # right_fit, right_fit_res, _, _, _ = np.polyfit(righty, rightx, 2, full=True)

    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )

    ### hyperparameters ###
    test = 0
    ###

    # experimental robust checks
    if test == 1 and len(lefty) > 2 and len(righty) > 2:
        # TODO:  polyfit and evaluate residuals, multiple passes for poor residuals using random sampling of
        # minimum of 500 or the len(lefty) or len(righty)
        n_samples = min(50, min( len(lefty), len(righty)))
        lefty_sampled = np.zeros(n_samples)
        righty_sampled = np.zeros(n_samples)
        leftx_sampled = np.zeros(n_samples)
        rightx_sampled = np.zeros(n_samples)
        for i in range(n_samples):  # draw random samples
            left_ind = np.random.random_integers(len(lefty)-1)
            lefty_sampled[i] = lefty[left_ind]
            leftx_sampled[i] = leftx[left_ind]
            right_ind = np.random.random_integers(len(righty)-1)
            righty_sampled[i] = righty[right_ind]
            rightx_sampled[i] = rightx[right_ind]

        fit_flag = 0
        res_tol = 50000  # usually 15,000 vs. 125,015
        lane_width = 200  # lane is 200 pixels wide
        lane_margin = 20  # +/- 20 pixel margin
        while fit_flag == 0:
            plt.pause(1)
            left_fit, left_fit_res, _, _, _ = np.polyfit(lefty_sampled, leftx_sampled, 2, full=True)
            right_fit, right_fit_res, _, _, _ = np.polyfit(righty_sampled, rightx_sampled, 2, full=True)

            if np.abs(left_fit_res) <= res_tol and np.abs(right_fit_res) > res_tol:  # use left line as a baseline, and get points around dx with margin for right
                print('left fix')
                left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
                x_min_ind = int(np.mean(left_fitx) + lane_width - lane_margin)
                x_max_ind = int(np.mean(left_fitx) + lane_width + lane_margin)
                # extract x, y values only in range - no upper limit this time for particles
                counter = 0
                for i in range(len(rightx)):
                    if counter == n_samples:  # if more samples in buffer
                        rightx_sampled = rightx_sampled[0:n_samples]
                        # fit_flag = 1
                        break
                    elif rightx[i] in range(x_min_ind, x_max_ind):  # if x is in range
                        rightx_sampled[counter] = rightx[i]
                        righty_sampled[counter] = righty[i]
                        counter = counter + 1
            elif np.abs(right_fit_res) <= res_tol and np.abs(left_fit_res) > res_tol:  # use right line as a baseline, and get points around dx with margin for left
                print('right fix')
                right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
                x_min_ind = int(np.mean(right_fitx) - lane_width - lane_margin)
                x_max_ind = int(np.mean(right_fitx) - lane_width + lane_margin)
                # extract x, y values only in range - no upper limit this time for particles
                counter = 0
                for i in range(len(leftx)):
                    if counter == n_samples:  # if more samples in buffer
                        leftx_sampled = leftx_sampled[0:n_samples]
                        # fit_flag = 1
                        break
                    elif leftx[i] in range(x_min_ind, x_max_ind):  # if x is in range
                        leftx_sampled[counter] = leftx[i]
                        lefty_sampled[counter] = lefty[i]
                        counter = counter + 1
            elif np.abs(right_fit_res) > res_tol and np.abs(left_fit_res) > res_tol:  # failure mode, relax constraint OR present naive lane
                fit_flag = 1  # break out of loop
            else:  # good condition
                fit_flag = 1
            fit_flag = 1
    else:
        left_fit, left_fit_res, _, _, _ = np.polyfit(lefty, leftx, 2, full=True)  # original implementation
        right_fit, right_fit_res, _, _, _ = np.polyfit(righty, rightx, 2, full=True)
        ### end TODO

    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    # out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    #
    # plt.imshow(out_img)
    # plt.plot(left_fitx, ploty, color='yellow')
    # plt.plot(right_fitx, ploty, color='yellow')
    # plt.xlim(0, 410)
    # plt.ylim(308, 0)

    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty

    return ploty, left_fit, right_fit, ret

# function not used
def skip_sliding_window(binary_warped, left_fit, right_fit):
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
    left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
    left_fit[1]*nonzeroy + left_fit[2] + margin)))

    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
    right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
    right_fit[1]*nonzeroy + right_fit[2] + margin)))

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    ###############################
    # Visualization
    ###############################

    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    window_img = np.zeros_like(out_img)
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                  ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin,
                                  ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

    plt.imshow(result)
    plt.plot(left_fitx, ploty, color='yellow')
    plt.plot(right_fitx, ploty, color='yellow')
    plt.xlim(0, 410)
    plt.ylim(308, 0)

    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty

    return ret

def measure_curvature(ploty, lines_info):

    # ### hyperparameters
    # ym_per_pix = (0.15/308)  # may need to change this based on bird's eye map
    # xm_per_pix = (0.15/410)
    # ###

    leftx = lines_info['left_fitx']
    rightx = lines_info['right_fitx']

    leftx = leftx[::-1]
    rightx = rightx[::-1]

    y_eval = np.max(ploty)
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # print(left_curverad, 'm', right_curverad, 'm')

    return left_curverad, right_curverad

def draw_lane_lines(original_image, warped_image, Minv, draw_info):
    leftx = draw_info['leftx']
    rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return result

def process_image(image, used_warped, used_ret):

    # apply gaussian blur
    image = cv2.GaussianBlur(image,(3,3),0)

    # crop image
    img_crop = crop_image(image)

    # Gradient thresholding
    gradient_combined = apply_thresholds(img_crop)

    # Color thresholding
    s_binary = apply_color_threshold(img_crop)

    # Combine Gradient and Color thresholding
    combined_binary = combine_threshold(s_binary, gradient_combined)

    # Transforming Perspective
    binary_warped, Minv = warp(combined_binary)

    # Getting Histogram
    histogram = get_histogram(binary_warped)

    # Sliding Window to detect lane lines
    ploty, left_fit, right_fit, ret = slide_window(binary_warped, histogram)

    # Skip sliding window option (not used here)
    # ret = skip_sliding_window(binary_warped, left_fit, right_fit)

    # Measuring Curvature
    left_curverad, right_curverad = measure_curvature(ploty, ret)

    # Sanity check: whether the lines are roughly parallel, have similar curvature, don't intersect, and have a minimum road width

    # ### hyperparameters
    # width_threshold = 50
    # slope_threshold = 150
    # curve_threshold = 20
    # ###

    slope_left = ret['left_fitx'][0] - ret['left_fitx'][-1]
    slope_right = ret['right_fitx'][0] - ret['right_fitx'][-1]

    x_left = ret['left_fitx'][-1]
    x_right = ret['right_fitx'][-1]

    road_width = np.abs(x_left - x_right)
    slope_diff = abs(slope_left - slope_right)
    curve_diff = abs(left_curverad - right_curverad)

    # print(slope_diff)
    # print(curve_diff)

    if (slope_diff > slope_threshold or curve_diff > curve_threshold or x_left > x_right or road_width < width_threshold):
        binary_warped = used_warped
        ret = used_ret

    # Visualizing Lane Lines Info
    result = draw_lane_lines(img_crop, binary_warped, Minv, ret)

    # Annotating curvature
    fontType = cv2.FONT_HERSHEY_SIMPLEX
    # curvature_text = 'The radius of curvature = ' + str(round(left_curverad, 3)) + 'm'
    # cv2.putText(result, curvature_text, (30, 60), fontType, 1.5, (255, 255, 255), 3)

    # Annotating deviation
    deviation_pixels = image.shape[1]/2 - abs(ret['right_fitx'][-1] - ret['left_fitx'][-1])

    # ### hyperparameters
    # xm_per_pix = (0.15/410)  # from manual inspection
    # ###

    deviation = deviation_pixels * xm_per_pix
    direction = "left" if deviation < 0 else "right"
    deviation_text = 'Vehicle is ' + str(round(abs(deviation), 3)) + 'm ' + direction + ' of center'
    # cv2.putText(result, deviation_text, (30, 110), fontType, 1.5, (255, 255, 255), 3)
    cv2.putText(result, deviation_text, (0,25), fontType, 0.5, (255, 255, 255), 1)

    used_warped = binary_warped
    used_ret = ret

    return result, used_warped, used_ret, deviation

if __name__ == "__main__":

    # initialization files (replace the directory with where you have the dataset)
    used_warped = plt.imread('/home/william/lane_detection/init.png')
    used_warped = apply_color_threshold(used_warped)
    used_ret = eval(open('/home/william/lane_detection/init.txt').read())

    manual = 0  # testing switch

    if manual == 0:
        # automatic workflow on test data
        for i in range(404):  # 1254
            print('Picture', i)
            filename = '/home/william/lane_detection/images/frame' + str(i).zfill(4) + '.jpg'
            image = mpimg.imread(filename)
            result_image, used_warped, used_ret, deviation = process_image(image, used_warped, used_ret)
            # plt.imsave('/home/william/lane_detection/images_robust_opt/' + str(i).zfill(4) + '.jpg', result_image)
            plt.clf()
            plt.imshow(result_image)
            plt.pause(0.00001)
    else:
        # manual workflow#
        image = mpimg.imread('/home/william/lane_detection/images/frame0300.jpg')
        image_org = image
        image = crop_image(image)
        # image= cv2.add(image,np.array([-10.0]))  # brighten image

        # apply gaussian blur
        image = cv2.GaussianBlur(image,(3,3),0)
        # plt.imshow(image)
        # plt.pause(60)

        combined = apply_thresholds(image)
        # compare_images(image, combined, "Original Image", "Gradient Thresholds")
        # plt.pause(60)

        s_binary = apply_color_threshold(image)
        # compare_images(image, s_binary, "Original Image", "Color Threshold")
        # plt.pause(60)

        combined_binary = combine_threshold(s_binary, combined)
        # compare_images(image, combined_binary, "Original Image", "Gradient and Color Threshold")
        # plt.pause(60)

        warped, Minv = warp(image)
        # compare_plotted_images(image, warped, "Original Image", "Warped Image")
        # plt.pause(60)

        binary_warped, Minv = warp(combined_binary)
        histogram = get_histogram(binary_warped)
        # plt.plot(histogram)
        # plt.pause(60)

        # ploty, left_fit, right_fit = slide_window(binary_warped, histogram)
        ploty, left_fit, right_fit, draw_info = slide_window(binary_warped, histogram)  # returning draw_info at this step
        # plt.pause(1e5)

        # draw_info = skip_sliding_window(binary_warped, left_fit, right_fit)
        # plt.pause(60)

        left_curverad, right_curverad = measure_curvature(ploty, draw_info)
        # plt.pause(60)

        result = draw_lane_lines(image, binary_warped, Minv, draw_info)
        plt.imshow(result)
        plt.pause(1e5)
