import numpy as np
import cv2
import matplotlib.pyplot as plt
import glob

### hyperparameters ###
# deviation_cutoff = 0.5  # cutoff the upper half of the x positions of the lanes
lane_bias = -0.03  # lane bias to center the robot
dashed_tol = 20  # number of pixels deviation for the max delta_y in the image allowed for turning into yellow lane
yellow_tol = 10  # minimum number of yellow pixels to be classified as a line
margin = 50  # margin given to sliding window width, tuned based on FOV
minpix = 30  # increased minpix for reduction of false positives
nwindows = 5  # number of sliding windows to bin pixels for binary warped image
xm_per_pix = (0.15/410)*(1/2.0)  # need to tune (used for curve_threshold calculation)
ym_per_pix = (0.15/308)*(1/2.0)  # may need to change this based on bird's eye map
img_width = 400  # cropped image width
img_height = 100  # croped image height
low = [20, 100, 100]  # lower hsv thresholds for yellow binning
high = [30, 255, 255]  # upper hsv thresholds for yellow binning

def crop_image(image):  # 410 x 308 image
    y = 208
    h = 308-y  # cover to the bottom
    x = 10
    w = 400
    img_crop = image[y:y+h, x:x+w]
    # plt.imshow(crop_img)  # for testing crop window sizes
    # plt.pause(0.0001)

    return img_crop

def apply_color_threshold(image):
    hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    # Compute a binary thresholded image where yellow is isolated from hsv components
    img_hsv_yellow_bin = np.zeros_like(hsv_img[:,:,0])
    img_hsv_yellow_bin[((hsv_img[:,:,0] >= low[0]) & (hsv_img[:,:,0] <= high[0])) \
                 & ((hsv_img[:,:,1] >= low[1]) & (hsv_img[:,:,1] <= high[1])) \
                 & ((hsv_img[:,:,2] >= low[2]) & (hsv_img[:,:,2] <= high[2])) \
                ] = 1

    # plt.imshow(img_hsv_yellow_bin)
    # plt.pause(0.1)

    # Combine white and yellow bins (right now only yellow)
    img_hsv_white_yellow_bin = np.zeros_like(hsv_img[:,:,0])
    img_hsv_white_yellow_bin[(img_hsv_yellow_bin == 1)] = 1 # only yellow bin

    # Lane classification
    yellow_pixels = np.nonzero(img_hsv_yellow_bin)  # return i,j indices (i.e. (y,x) )
    if np.count_nonzero(img_hsv_yellow_bin) < yellow_tol:  # if there are no yellow pixels seen (default to regular astar)
        road_side_flag = 'none'
    elif np.mean(yellow_pixels[1]) > img_width/2.0:  # yellow lane is in the right side of the robot heading
        road_side_flag = 'left'
    else:  # yellow lane is in the left side
        road_side_flag = 'right'

    lane_flag = 'normal'  # used if yellow lanes are dashed

    color_flags = [road_side_flag, lane_flag]

    return img_hsv_white_yellow_bin, color_flags

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

def lane_check(combined_binary):  # fits line through the binary image and determines slope for lane direction
    pixelpoints = np.nonzero(combined_binary)  # pixelpoints is tuple of array (return (i,j) indices i.e. (y,x) )
    p = np.polyfit(pixelpoints[1], pixelpoints[0], 1)
    if p[0] < 0:  # line slope is negative, and thus is a left turn
        return -1
    else:
        return 1

def slide_window(binary_warped, histogram):  # change sliding window to only one lane detection
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    # midpoint = np.int(histogram.shape[0]/2)
    # leftx_base = np.argmax(histogram[:midpoint])
    # rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    leftx_base = np.argmax(histogram[:])  # start at the max of the histogram (should be just yellow)

    # nwindows = 5  # lowered number of windows based on FOV
    window_height = np.int(binary_warped.shape[0]/nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    # rightx_current = rightx_base

    # ### hyperparameters ###
    # margin = 50  # tuned based on FOV
    # minpix = 50  # increased minpix for reduction of false positives
    # ###

    left_lane_inds = []
    # right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        # win_xright_low = rightx_current - margin
        # win_xright_high = rightx_current + margin
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
        (0,255,0), 2)
        # cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
        # (0,255,0), 2)
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        # good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        # (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        # right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        # if len(good_right_inds) > minpix:
        #     rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    # right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    # rightx = nonzerox[right_lane_inds]
    # righty = nonzeroy[right_lane_inds]

    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )

    # print(lefty)

    if np.size(lefty) > 2:
        left_fit, left_fit_res, _, _, _ = np.polyfit(lefty, leftx, 2, full=True)  # original implementation
        # right_fit, right_fit_res, _, _, _ = np.polyfit(righty, rightx, 2, full=True)

        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        # right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        right_fit = 0
        fit_flag = 1
    else:
        left_fit = 0
        right_fit = 0
        left_fitx = 0
        right_fitx = 0
        fit_flag = 0

    # out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    #
    # plt.imshow(out_img)
    # plt.plot(left_fitx, ploty, color='yellow')
    # # plt.plot(right_fitx, ploty, color='yellow')
    # plt.xlim(0, 410)
    # plt.ylim(308, 0)

    ret = {}
    ret['leftx'] = leftx
    # ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    # ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty

    return ploty, left_fit, right_fit, ret, fit_flag

def draw_lane_lines(original_image, warped_image, Minv, draw_info):
    leftx = draw_info['leftx']
    # rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    # right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    # pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    # pts = np.hstack((pts_left, pts_right))
    pts = pts_left

    cv2.fillPoly(color_warp, np.int_([pts]), (0,255,0))

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return result

def world_transform(x_pts, y_pts):  # transform from camera frame to robot frame in world
    x_rot = np.zeros((np.size(x_pts),1))
    y_rot = np.zeros((np.size(y_pts),1))
    rot_mat = np.array([ [0, 1, 0], [-1, 0, 0], [0, 0, 1] ])
    for i in range(np.size(x_pts)):
        rot_vec = np.matmul(rot_mat, np.array([ x_pts[i], y_pts[i], 0 ])) + np.array([ -img_width/2.0, img_height, 0 ])
        x_rot[i] = rot_vec[0]
        y_rot[i] = rot_vec[1]

    return x_rot, y_rot

def process_image(image, used_warped, used_ret):

    # Crop image
    img_crop = crop_image(image)

    # Apply gaussian blur
    img_crop = cv2.GaussianBlur(img_crop,(3,3),0)

    # Color thresholding
    combined_binary, color_flags = apply_color_threshold(img_crop)
    # combined_binary = s_binary

    # plt.imshow(s_binary)
    # plt.pause(30)

    # Immediate flag checks for failure cases
    road_side_flag, lane_flag = color_flags[0], color_flags[1]

    if road_side_flag == 'none':  # if no yellow lane detected
        # print('No Yellow Lane')
        binary_warped = used_warped
        ret = used_ret
        result = img_crop
        lane_deviation = 0
    else:
        # Transforming Perspective
        binary_warped, Minv = warp(combined_binary)

        # Getting Histogram
        histogram = get_histogram(binary_warped)

        # Sliding Window to detect lane lines
        ploty, left_fit, right_fit, ret, fit_flag = slide_window(binary_warped, histogram)

        # Map the (x,y) map and project yellow lanes onto the occupancy gridss
        if fit_flag == 1:
            x_left = ret['left_fitx'] - img_width/2.0  # pixel
            y_left = ploty  # pixel

            x_left = np.resize(x_left, (1,len(x_left)))
            y_left = np.resize(y_left, (1,len(y_left)))

            # # extend all the way to the left
            # x_min = np.amin(x_left)  # in pixel coordinates
            # x_block = np.arange(-img_width/2.0, x_min, grid_res/(4*xm_per_pix))  # in pixel coordinates
            # y_block = (grid_res/(4*ym_per_pix))*np.ones((np.size(x_block),1))  # in pixel coordinates

            # x_left = np.vstack((np.resize(x_block, (len(x_block),1)), x_left))
            # y_left = np.vstack((y_block, y_left))

            # lane_ind = np.hstack((xm_per_pix*x_left, ym_per_pix*y_left))  # Nx2 numpy array for the yellow lanes

            result = draw_lane_lines(img_crop, binary_warped, Minv, ret)

            if np.amax(np.abs(np.diff(y_left)) > dashed_tol):
                lane_deviation = 0  # dashed line tolerance check
                lane_flag = 'dashed'
            else:
                lane_deviation = xm_per_pix*np.mean(x_left) + lane_bias  # main return

            # pixels = np.nonzero(binary_warped)  # extract the binary coordinates
            # u_rot, v_rot = world_transform(pixels[0], pixels[1])

            # # Plotting for world-frame comparison
            # plt.plot(lane_ind[:,0], lane_ind[:,1],'.')
            # plt.hold(True)
            # plt.plot(u_rot*xm_per_pix, v_rot*ym_per_pix,'o')
            # plt.xlim([-0.15/2, 0.15/2])
            # plt.ylim([0, 0.15/2])
            # plt.pause(600)

        else:  # no fit available, in cases where the road_side_flag fails
            binary_warped = used_warped
            ret = used_ret
            result = img_crop
            lane_deviation = 0

        used_warped = binary_warped
        used_ret = ret

        result = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)

    return result, used_warped, used_ret, lane_deviation, road_side_flag, lane_flag



###
if __name__ == "__main__":

    # initialization files (replace the directory with where you have the dataset)
    used_warped = cv2.imread('/home/william/lane_detection/init.png')
    used_warped = apply_color_threshold(used_warped)
    used_ret = eval(open('/home/william/lane_detection/init.txt').read())

    manual = 1  # testing switch

    if manual == 0:
        # automatic workflow on test data
        for i in range(404):  # 1254
            print('Picture', i)
            filename = '/home/william/lane_detection/images/frame' + str(i).zfill(4) + '.jpg'
            image = cv2.imread(filename)
            result_image, used_warped, used_ret, deviation = process_image(image, used_warped, used_ret)
            # plt.imsave('/home/william/lane_detection/images_robust_opt/' + str(i).zfill(4) + '.jpg', result_image)
            plt.clf()
            # plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            cv2.imshow(result_image)
            plt.pause(0.00001)
    else:
        # manual workflow#
        image = plt.imread('/home/william/lane_detection/images/frame0300.jpg')  # read in as RGB
        # image_org = image
        # image= cv2.add(image,np.array([-10.0]))  # brighten image

        # Crop image
        img_crop = crop_image(image)

        # Apply gaussian blur
        img_crop = cv2.GaussianBlur(img_crop,(3,3),0)

        # Color thresholding
        s_binary, color_flags = apply_color_threshold(img_crop)
        combined_binary = s_binary

        # plt.imshow(s_binary)
        # plt.pause(30)

        # Immediate flag checks for failure cases
        road_side_flag, lane_flag = color_flags[0], color_flags[1]

        if road_side_flag == 'none':  # if no yellow lane detected, default to regular move cost
            print('No Yellow Lane')
            binary_warped = used_warped
            ret = used_ret
            result = img_crop
            lane_ind = None
        else:
            # Transforming Perspective
            binary_warped, Minv = warp(combined_binary)

            # Getting Histogram
            histogram = get_histogram(binary_warped)

            # Sliding Window to detect lane lines
            ploty, left_fit, right_fit, ret, fit_flag = slide_window(binary_warped, histogram)

            # Map the (x,y) map and project yellow lanes onto the occupancy gridss
            if fit_flag == 1:
                x_left = ret['left_fitx'] - img_width/2.0
                y_left = ploty

                x_left = np.resize(x_left, (len(x_left),1))
                y_left = np.resize(y_left, (len(y_left),1))

                # extend all the way to the left
                x_min = np.amin(x_left)  # in pixel coordinates
                x_block = np.arange(-img_width/2.0, x_min, grid_res/(4*xm_per_pix))  # in pixel coordinates
                y_block = (grid_res/(4*ym_per_pix))*np.ones((np.size(x_block),1))  # in pixel coordinates

                x_left = np.vstack((np.resize(x_block, (len(x_block),1)), x_left))
                y_left = np.vstack((y_block, y_left))

                lane_ind = np.hstack((xm_per_pix*x_left, ym_per_pix*y_left))  # Nx2 numpy array for the yellow lanes

                result = draw_lane_lines(img_crop, binary_warped, Minv, ret)

                pixels = np.nonzero(binary_warped)  # extract the binary coordinates
                u_rot, v_rot = world_transform(pixels[0], pixels[1])

                # # Plotting for world-frame comparison
                # plt.plot(lane_ind[:,0], lane_ind[:,1],'.')
                # plt.hold(True)
                # plt.plot(u_rot*xm_per_pix, v_rot*ym_per_pix,'o')
                # plt.xlim([-0.15/2, 0.15/2])
                # plt.ylim([0, 0.15/2])
                # plt.pause(600)

            else:  # no fit available, in cases where the road_side_flag fails
                binary_warped = used_warped
                ret = used_ret
                result = img_crop
                lane_ind = None

            used_warped = binary_warped
            used_ret = ret
