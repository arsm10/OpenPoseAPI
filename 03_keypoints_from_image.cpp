// ----------------------- OpenPose C++ API Tutorial - Example 3 - Body from image -----------------------
// It reads an image, process it, and displays it with the pose (and optionally hand and face) keypoints. In addition,
// it includes all the OpenPose configuration flags (enable/disable hand, face, output saving, etc.).

// Third-party dependencies
#include <opencv2/opencv.hpp>
// Command-line user interface
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

// csvfile Generation
ofstream csvFile("../Results/CSVFiles/KeyPointsPose_1.csv"), csvAngle ("../Results/CSVFiles/JointAnglesPose_1.csv");
VideoWriter detectedVideo("../Results/OpenPoseVideos/DetectionPose_1.avi.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(640, 352));
//initialization of Frame
int frame = 1;
std::vector<cv::Point2f> neckJoint, rightShoulder, rightElbow, leftShoulder, leftElbow, hipJoint, virtHipJoint, rightHip, rightKnee, rightAnkle, leftHip, leftKnee, leftAnkle, u, v;
float rightShoulderAbduction, leftShoulderAbduction, rightShoulderFlexion, leftShoulderFlexion, trunckInclination, rightKneeAngle, leftKneeAngle;

// Custom OpenPose flags
// Producer
DEFINE_string(image_path, "examples/media/COCO_val2014_000000000294.jpg",
    "Process an image. Read all standard formats (jpg, png, bmp, etc.).");
// Display
DEFINE_bool(no_display,                 false,
    "Enable to disable the visual display.");

//OpenPose Joint Name
string JointName(int type)
{
	string jName;

	switch (type)
	{
	case 0: return jName = "Nose";
	case 1:	return jName = "Neck";
	case 2:	return jName = "Right Shoulder";
	case 3: return jName = "Right Elbow";
	case 4: return jName = "Right Wrist";
	case 5: return jName = "Left Shoulder";
	case 6: return jName = "Left Elbow";
	case 7: return jName = "Left Wrist";
	case 8: return jName = "Mid Hip";
	case 9: return jName = "Right Hip";
	case 10: return jName = "Right Knee";
	case 11: return jName = "Right Ankle";
	case 12: return jName = "Left Hip";
	case 13: return jName = "Left Knee";
	case 14: return jName = "Left Ankle";
	case 15: return jName = "Right Eye";
	case 16: return jName = "Left Eye";
	case 17: return jName = "Right Ear";
	case 18: return jName = "Left Ear";
	case 19: return jName = "Left Big Toe";
	case 20: return jName = "Left Small Toe";
	case 21: return jName = "Left Heel";
	case 22: return jName = "Right Big Toe";
	case 23: return jName = "Right Small Toe";
	case 24: return jName = "Right Heel";
	default:
		return jName = "unknown joint";
	}
}

//Joint vectors
vector<cv::Point2f> Joint_Axes(float x_axes, float y_axes)
{
	vector<cv::Point2f> joint_coord;
	joint_coord.push_back(Point2f(x_axes, y_axes));
	return joint_coord;
}

//Angle calclation
float JointAngle(std::vector<cv::Point2f> vector_A, std::vector<cv::Point2f> vector_B)
{
	float dot_product = (vector_A[0].x * vector_B[0].x) + (vector_A[0].y * vector_B[0].y);
	float u_len = sqrtf(powf(vector_A[0].x, 2.0) + powf(vector_A[0].y, 2.0));
	float v_len = sqrtf(powf(vector_B[0].x, 2.0) + powf(vector_B[0].y, 2.0));
	float length = u_len * v_len;
	float angle = (acosf(dot_product / length) * 180 / 3.14159265);
	return angle;
}
// This worker will just read and return all the jpg files in a directory
void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // User's displaying/saving/other processing here
            // datum.cvOutputData: rendered frame with pose or heatmaps
            // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
			/*cv::putText(cvMat, "Right Shoulder Abduction: " + std::to_string(rightShoulderAbduction), cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, 5, false);
			cv::putText(cvMat, "Trunk Inclination: " + std::to_string(trunckInclination), cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, 5, false);
			cv::putText(cvMat, "Right Knee Flexion: " + std::to_string(rightKneeAngle), cv::Point(20, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, 5, false);
			cv::putText(cvMat, "Left Shoulder Abduction: " + std::to_string(leftShoulderAbduction), cv::Point(20, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, 5, false);
			cv::putText(cvMat, "Left Knee Flexion: " + std::to_string(leftKneeAngle), cv::Point(20, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, 5, false);*/
			cv::putText(cvMat, "Left Shoulder Flexion: " + std::to_string(leftShoulderFlexion), cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, 5, false);
			cv::putText(cvMat, "Right Shoulder Flexion: " + std::to_string(rightShoulderFlexion), cv::Point(20, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 255), 0.5, 5, false);
            cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
			detectedVideo.write(cvMat);
            cv::waitKey(10);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
		//increment in frame 
		int frame_number = frame++;
		
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
			op::opLog("Keypoints of Joints: Frame,Joint Name, X, Y, Confidence Level", op::Priority::High);
			for (size_t i = 0; i < datumsPtr->at(0)->poseKeypoints.getVolume(); i += 3)
			{
				cout << frame_number << "," << JointName(i/3) << ", "<< datumsPtr->at(0)->poseKeypoints.at(i) << ", " << datumsPtr->at(0)->poseKeypoints.at(i + 1) << ", " << datumsPtr->at(0)->poseKeypoints.at(i + 2) << endl;
				csvFile << frame_number << "," << JointName(i/3) << "," << datumsPtr->at(0)->poseKeypoints.at(i) << "," << datumsPtr->at(0)->poseKeypoints.at(i + 1) << "," << datumsPtr->at(0)->poseKeypoints.at(i + 2) << "\n";

				if (JointName(i / 3) == "Neck") {
					neckJoint = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));
				}
				if (JointName(i / 3) == "Right Shoulder") {
					rightShoulder = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));
				}
				if (JointName(i / 3) == "Right Elbow") {
					rightElbow = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));

					//Right Shoulder Abduction
					u = Joint_Axes(rightShoulder[0].x - neckJoint[0].x, rightShoulder[0].y - neckJoint[0].y);
					v = Joint_Axes(rightShoulder[0].x - rightElbow[0].x, rightShoulder[0].y - rightElbow[0].y);

					rightShoulderAbduction = JointAngle(u, v);
					if (rightShoulder[0].y < rightElbow[0].y) {
						rightShoulderAbduction = rightShoulderAbduction - 90;
					}
					if (rightShoulder[0].y >= rightElbow[0].y) {
						rightShoulderAbduction = 270 - rightShoulderAbduction;
					}
					csvAngle << frame_number << "," << "Right Shoulder Abduction" << "," << rightShoulderAbduction << "\n";

					u.clear();
					v.clear();
				}
				if (JointName(i / 3) == "Left Shoulder") {
					leftShoulder = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));
				}
				if (JointName(i / 3) == "Left Elbow") {
					leftElbow = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));

					//Left Shoulder Abduction
					u = Joint_Axes(leftShoulder[0].x - neckJoint[0].x, leftShoulder[0].y - neckJoint[0].y);
					v = Joint_Axes(leftShoulder[0].x - leftElbow[0].x, leftShoulder[0].y - leftElbow[0].y);

					leftShoulderAbduction = JointAngle(u, v);
					if (leftShoulder[0].y < leftElbow[0].y) {
						leftShoulderAbduction = leftShoulderAbduction - 90;
					}
					if (leftShoulder[0].y >= leftElbow[0].y) {
						leftShoulderAbduction = 270 - leftShoulderAbduction;
					}
					csvAngle << frame_number << "," << "Left Shoulder Abduction" << "," << leftShoulderAbduction << "\n";

					u.clear();
					v.clear();
				}
				if (JointName(i / 3) == "Mid Hip") {
					hipJoint = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));
					virtHipJoint = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1)-100);

					//Trunk Inclination
					u = Joint_Axes(neckJoint[0].x - hipJoint[0].x, neckJoint[0].y - hipJoint[0].y);
					v = Joint_Axes(virtHipJoint[0].x - hipJoint[0].x, virtHipJoint[0].y - hipJoint[0].y);

					trunckInclination = JointAngle(u, v);
					csvAngle << frame_number << "," << "Trunk Inclination" << "," << trunckInclination << "\n";

					u.clear();
					v.clear();
				}
				if (JointName(i / 3) == "Right Hip") {
					rightHip = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));

					//Right Shoulder Flexion
					u = Joint_Axes(rightHip[0].x - rightShoulder[0].x, rightHip[0].y - rightShoulder[0].y);
					v = Joint_Axes(rightElbow[0].x - rightShoulder[0].x, rightElbow[0].y - rightShoulder[0].y);

					rightShoulderFlexion = JointAngle(u, v);
					csvAngle << frame_number << "," << "Right Shoulder Flexion" << "," << rightShoulderFlexion << "\n";

					u.clear();
					v.clear();
				}
				if (JointName(i / 3) == "Right Knee") {
					rightKnee = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));
				}
				if (JointName(i / 3) == "Right Ankle") {
					rightAnkle = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));

					//Right Knee Flexion
					u = Joint_Axes(rightHip[0].x - rightKnee[0].x, rightHip[0].y - rightKnee[0].y);
					v = Joint_Axes(rightAnkle[0].x - rightKnee[0].x, rightAnkle[0].y - rightKnee[0].y);

					rightKneeAngle = JointAngle(u, v);
					csvAngle << frame_number << "," << "Right Knee Flexion" << "," << rightKneeAngle << "\n";

					u.clear();
					v.clear();
				}
				if (JointName(i / 3) == "Left Hip") {
					leftHip = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));

					//Left Shoulder Flexion
					u = Joint_Axes(leftHip[0].x - leftShoulder[0].x, leftHip[0].y - leftShoulder[0].y);
					v = Joint_Axes(leftElbow[0].x - leftShoulder[0].x, leftElbow[0].y - leftShoulder[0].y);

					leftShoulderFlexion = JointAngle(u, v);
					csvAngle << frame_number << "," << "Left Shoulder Flexion" << "," << leftShoulderFlexion << "\n";

					u.clear();
					v.clear();
				}
				if (JointName(i / 3) == "Left Knee") {
					leftKnee = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));
				}
				if (JointName(i / 3) == "Left Ankle") {
					leftAnkle = Joint_Axes(datumsPtr->at(0)->poseKeypoints.at(i), datumsPtr->at(0)->poseKeypoints.at(i + 1));

					//Left Knee Flexion
					u = Joint_Axes(leftHip[0].x - leftKnee[0].x, leftHip[0].y - leftKnee[0].y);
					v = Joint_Axes(leftAnkle[0].x - leftKnee[0].x, leftAnkle[0].y - leftHip[0].y);

					leftKneeAngle = JointAngle(u, v);
					csvAngle << frame_number << "," << "Left Knee Flexion" << "," << leftKneeAngle << "\n";

					u.clear();
					v.clear();
				}
				neckJoint.clear();
				rightShoulder.clear();
				rightElbow.clear();
				leftShoulder.clear();
				leftElbow.clear();
				hipJoint.clear();
				virtHipJoint.clear();
				rightHip.clear();
				rightKnee.clear();
				rightAnkle.clear();
				leftHip.clear();
				leftKnee.clear();
				leftAnkle.clear();
			}

            /*op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
            op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
            op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
            op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);*/
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void configureWrapper(op::Wrapper& opWrapper)
{
    try
    {
        // Configuring OpenPose

        // logging_level
        op::checkBool(
            0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
            __LINE__, __FUNCTION__, __FILE__);
        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

        // Applying user defined configuration - GFlags to program variables
        // outputSize
        const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
        // netInputSize
        const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
        // faceNetInputSize
        const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
        // handNetInputSize
        const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
        // poseModel
        const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            op::opLog(
                "Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
                " instead.", op::Priority::Max);
        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        // >1 camera view?
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{
            poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
            FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
            poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
            FLAGS_part_to_show, op::String(FLAGS_model_folder), heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
            (float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
            op::String(FLAGS_prototxt_path), op::String(FLAGS_caffemodel_path),
            (float)FLAGS_upsampling_ratio, enableGoogleLogging};
        opWrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
        opWrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
        opWrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
        opWrapper.configure(wrapperStructExtra);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port)};
        opWrapper.configure(wrapperStructOutput);
        // No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int tutorialApiCpp()
{
    try
    {
		//Path of the Video
		VideoCapture test_video("../InputVideos/Pose_4.mp4");

        op::opLog("Starting OpenPose demo...", op::Priority::High);
        const auto opTimer = op::getTimerInit();

        // Configuring OpenPose
        op::opLog("Configuring OpenPose...", op::Priority::High);
        op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
        configureWrapper(opWrapper);

        // Starting OpenPose
        op::opLog("Starting thread(s)...", op::Priority::High);
        opWrapper.start();

		if (!test_video.isOpened())
		{
			std::cout << "File is already Opened" << endl;
			return -1;
		}
		// Process and Disply Video
		while (1)
		{
			cv::Mat videoProces;
			//Capture frame-by-frame
			test_video >> videoProces;
			op::Matrix videoToProcess = OP_CV2OPMAT(videoProces);
			auto datumProcessed = opWrapper.emplaceAndPop(videoToProcess);

			//Condition when video is finished process is break
			if (videoProces.empty())
			{
				break;
			}
			if (datumProcessed != nullptr)
			{
				//Show video
				imshow("Video", videoProces);
				//Generate KeyPoints
				printKeypoints(datumProcessed);
				display(datumProcessed);
				/*if (!FLAGS_no_display)
				{
					display(datumProcessed);
				}		    */
			}


			char c = (char)waitKey(25);
			if (c == 27)
			{
				break;
			}
		}
		test_video.release();
		destroyAllWindows();

        // Process and display image
        /*const cv::Mat cvImageToProcess = cv::imread(FLAGS_image_path);
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
        auto datumProcessed = opWrapper.emplaceAndPop(imageToProcess);
        if (datumProcessed != nullptr)
        {
            printKeypoints(datumProcessed);
            if (!FLAGS_no_display)
                display(datumProcessed);
        }
        else
            op::opLog("Image could not be processed.", op::Priority::High);*/

        // Measuring total time
        op::printTime(opTimer, "OpenPose demo successfully finished. Total time: ", " seconds.", op::Priority::High);

        // Return
        return 0;
    }
    catch (const std::exception&)
    {
        return -1;
    }
}

int main(int argc, char *argv[])
{
    // Parsing command line flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);

	//Column Names of CSV file
	csvFile << "Frame" << "," << "Joint Name" << "," << "X" << "," << "Y" << "," << "Confidence Level" << "\n";
	csvAngle << "Frame" << "," << "Joint Name" << "," << "Angle (Deg)" << "\n";

    // Running tutorialApiCpp
    return tutorialApiCpp();
}
