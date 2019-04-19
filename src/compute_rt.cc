//
// Created by zzz on 19-4-18.
//

#include "compute_rt.h"
namespace GS{
  ComputeRT::ComputeRT(){
    pkg_loc = ros::package::getPath("lidar_camera_calibration");
    iteration_counter = 0;
  }

  Eigen::Quaterniond ComputeRT::addQ(Eigen::Quaterniond a, Eigen::Quaterniond b){
    Eigen::Quaterniond retval;
    if(a.x()*b.x() + a.y()*b.y() + a.z()*b.z() + a.w()*b.w() < 0.0)
    {
      b.x() = -b.x();
      b.y() = -b.y();
      b.z() = -b.z();
      b.w() = -b.w();
    }
    retval.x() = a.x() + b.x();
    retval.y() = a.y() + b.y();
    retval.z() = a.z() + b.z();
    retval.w() = a.w() + b.w();
    return retval;
  }

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ComputeRT::readArray(){
    std::ifstream infile(pkg_loc + "/conf/points.txt");
    int num_points=0;

    infile >> num_points;

    ROS_ASSERT(num_points > 0);

    Eigen::MatrixXd lidar(3,num_points), camera(3,num_points);

    std::cout << "Num points is:" << num_points << std::endl;

    for(int i=0; i<num_points; i++)
    {
      infile >> lidar(0,i) >> lidar(1,i) >> lidar(2,i);
    }
    for(int i=0; i<num_points; i++)
    {
      infile >> camera(0,i) >> camera(1,i) >> camera(2,i);
    }
    infile.close();

    // camera values are stored in variable 'lidar' and vice-versa
    // need to change this
    return std::pair<Eigen::MatrixXd, Eigen::MatrixXd>(lidar, camera);
  };

  Eigen::Matrix4d ComputeRT::calc_RT(Eigen::MatrixXd lidar, Eigen::MatrixXd camera,
                                     int MAX_ITERS, Eigen::Matrix3d lidarToCamera)
  {
    if(iteration_counter == 0)
    {
      std::ofstream clean_file(pkg_loc + "/log/avg_values.txt", std::ios_base::trunc);
      clean_file.close();

      translation_sum << 0.0, 0.0, 0.0;
      rotation_sum = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
      rotation_avg_by_mult << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;
      rmse_avg = 0.0;
    }
    int num_points = lidar.cols();
    std::cout << "Number of points: " << num_points << std::endl;
    Eigen::Vector3d mu_lidar, mu_camera;

    mu_lidar << 0.0, 0.0, 0.0;
    mu_camera << 0.0, 0.0, 0.0;

    for(int i=0; i<num_points; i++)
    {
      mu_lidar(0) += lidar(0,i);
      mu_lidar(1) += lidar(1,i);
      mu_lidar(2) += lidar(2,i);
    }
    for(int i=0; i<num_points; i++)
    {
      mu_camera(0) += camera(0,i);
      mu_camera(1) += camera(1,i);
      mu_camera(2) += camera(2,i);
    }

    mu_lidar = mu_lidar/num_points;
    mu_camera = mu_camera/num_points;

    if(iteration_counter == 0)
    {
      std::cout << "mu_lidar: \n" << mu_lidar << std::endl;
      std::cout << "mu_camera: \n" << mu_camera << std::endl;
    }

    Eigen::MatrixXd lidar_centered = lidar.colwise() - mu_lidar;
    Eigen::MatrixXd camera_centered = camera.colwise() - mu_camera;

    if(iteration_counter == 0)
    {
      std::cout << "lidar_centered: \n" << lidar_centered << std::endl;
      std::cout << "camera_centered: \n" << camera_centered << std::endl;
    }

    Eigen::Matrix3d cov = camera_centered*lidar_centered.transpose();

    std::cout << cov << std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d rotation;
    rotation = svd.matrixU() * svd.matrixV().transpose();
    if( rotation.determinant() < 0 )
    {
      Eigen::Vector3d diag_correct;
      diag_correct << 1.0, 1.0, -1.0;

      rotation = svd.matrixU() * diag_correct.asDiagonal() * svd.matrixV().transpose();
    }

    Eigen::Vector3d translation = mu_camera - rotation*mu_lidar;

    // averaging translation and rotation
    translation_sum += translation;
    Eigen::Quaterniond temp_q(rotation);
    rotation_sum = addQ(rotation_sum, temp_q);
    if(iteration_counter==0){
      initial_rotation = temp_q;
      initial_translation = translation;
    }
    // averaging rotations by multiplication
    rotation_avg_by_mult = rotation_avg_by_mult.pow(1.0*iteration_counter/(iteration_counter+1))*rotation.pow(1.0/(iteration_counter+1));

    Eigen::Vector3d ea = rotation.eulerAngles(2, 1, 0);

    std::cout << "Rotation matrix: \n" << rotation << std::endl;
    std::cout << "Rotation in Euler angles: \n" << ea*57.3 << std::endl;
    std::cout << "Translation: \n" << translation << std::endl;

    Eigen::MatrixXd eltwise_error = (camera - ((rotation*lidar).colwise() + translation)).array().square().colwise().sum();
    double error = sqrt(eltwise_error.sum()/num_points);
    std::cout << "RMSE: " << error << std::endl;

    rmse_avg = rmse_avg + error;

    Eigen::Matrix4d T;
    T.setIdentity(4,4);
    T.topLeftCorner(3, 3) = rotation;
    T.col(3).head(3) = translation;

    std::cout << "Rigid-body transformation: \n" << T << std::endl;

    iteration_counter++;
    //if(iteration_counter == MAX_ITERS)
    if(iteration_counter%1 == 0)
    {
      std::ofstream log_avg_values(pkg_loc + "/log/avg_values.txt", std::ios_base::app);

      std::cout << "--------------------------------------------------------------------\n";
      std::cout << "After " << iteration_counter << " iterations\n";
      std::cout << "--------------------------------------------------------------------\n";

      std::cout << "Average translation is:" << "\n" << translation_sum/iteration_counter << "\n";
      log_avg_values << iteration_counter << "\n";
      log_avg_values << translation_sum/iteration_counter << "\n";


      rotation_sum.x() = rotation_sum.x()/iteration_counter;
      rotation_sum.y() = rotation_sum.y()/iteration_counter;
      rotation_sum.z() = rotation_sum.z()/iteration_counter;
      rotation_sum.w() = rotation_sum.w()/iteration_counter;
      double mag = sqrt(rotation_sum.x()*rotation_sum.x() +
                        rotation_sum.y()*rotation_sum.y() +
                        rotation_sum.z()*rotation_sum.z() +
                        rotation_sum.w()*rotation_sum.w());
      rotation_sum.x() = rotation_sum.x()/mag;
      rotation_sum.y() = rotation_sum.y()/mag;
      rotation_sum.z() = rotation_sum.z()/mag;
      rotation_sum.w() = rotation_sum.w()/mag;

      Eigen::Matrix3d rotation_avg = rotation_sum.toRotationMatrix();
      std::cout << "Average rotation is:" << "\n" << rotation_avg << "\n";
      Eigen::Matrix3d final_rotation = rotation_avg * lidarToCamera;
      Eigen::Vector3d final_angles = final_rotation.eulerAngles(2, 1, 0);

      //std::cout << "Average rotation by multiplication is:" << "\n" << rotation_avg_by_mult << "\n";

      /*std::cout      << rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
                     << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
                     << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";*/

      log_avg_values << std::fixed << std::setprecision(8)
                     << rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
                     << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
                     << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";

      Eigen::Matrix4d T;
      T.setIdentity(4,4);
      T.topLeftCorner(3, 3) = final_rotation;
      T.col(3).head(3) = translation_sum/iteration_counter;
      std::cout << "Average Lidar to Camera transformation is: \n" << T << "\n";
      std::cout << "Final Lidar to Camera translation is:" << "\n" << T.col(3).head(3).transpose() << "\n";
      std::cout << "Final Lidar to Camera rotation is:" << "\n" << final_rotation << "\n";
      std::cout << "Final ypr is:" << "\n" <<final_angles.transpose() << "\n";

      Eigen::Matrix4d T_lidar_camera = T.inverse();
      Eigen::Vector3d ypr_lidar_camera = T_lidar_camera.block<3,3>(0,0).eulerAngles(2, 1, 0);
      std::cout << "Final Camera to Lidar translation is:" << "\n" << T_lidar_camera.col(3).head(3).transpose() << "\n";
      std::cout << "Final Camera to Lidar ypr is:" << "\n" << ypr_lidar_camera.transpose() << "\n";
      std::cout << "Average RMSE is: " <<  rmse_avg*1.0/iteration_counter << "\n";

      Eigen::MatrixXd eltwise_error_temp = (camera - ((rotation_avg*lidar).colwise() + (translation_sum/iteration_counter))).array().square().colwise().sum();
      double error_temp = sqrt(eltwise_error_temp.sum()/num_points);

      std::cout << "RMSE on average transformation is: " << error_temp << std::endl;
      log_avg_values << std::fixed << std::setprecision(8) << error_temp << "\n";
//		{
//			tf2_ros::TransformBroadcaster br;
//			geometry_msgs::TransformStamped transformStamped;
//			transformStamped.header.frame_id = "rslidar";
//			transformStamped.child_frame_id = "camera_link";
//			transformStamped.transform.translation.x = T_lidar_camera(0,3);
//			transformStamped.transform.translation.y = T_lidar_camera(1,3);
//			transformStamped.transform.translation.z = T_lidar_camera(2,3);
//			tf2::Quaternion q;
//			q.setRPY(ypr_lidar_camera(2), ypr_lidar_camera(1), ypr_lidar_camera(0));
//			transformStamped.transform.rotation.x = q.x();
//			transformStamped.transform.rotation.y = q.y();
//			transformStamped.transform.rotation.z = q.z();
//			transformStamped.transform.rotation.w = q.w();
//			br.sendTransform(transformStamped);
//		}

    }

    //writing files to generate plots

    /*if(iteration_counter%1 == 0)
   {
       std::ofstream log_avg_values(pkg_loc + "/log/avg_values.txt", std::ios_base::app);

       log_avg_values << iteration_counter << "\n";
       log_avg_values << translation_sum/iteration_counter << "\n";

       double mag = sqrt(rotation_sum.x()*rotation_sum.x() +
                    rotation_sum.y()*rotation_sum.y() +
                    rotation_sum.z()*rotation_sum.z() +
                    rotation_sum.w()*rotation_sum.w());

       Eigen::Quaterniond rot_temp_sum;
       rot_temp_sum.x() = rotation_sum.x()/(mag*iteration_counter);
       rot_temp_sum.y() = rotation_sum.y()/(mag*iteration_counter);
       rot_temp_sum.z() = rotation_sum.z()/(mag*iteration_counter);
       rot_temp_sum.w() = rotation_sum.w()/(mag*iteration_counter);

       Eigen::Matrix3d rotation_avg = rot_temp_sum.toRotationMatrix();
       //log_avg_values << rotation_avg << "\n";

       log_avg_values << std::fixed << std::setprecision(8)
                       << rotation_avg(0,0) << " " << rotation_avg(0,1) << " " << rotation_avg(0,2) << "\n"
                      << rotation_avg(1,0) << " " << rotation_avg(1,1) << " " << rotation_avg(1,2) << "\n"
                      << rotation_avg(2,0) << " " << rotation_avg(2,1) << " " << rotation_avg(2,2) << "\n";

       MatrixXd eltwise_error_temp = (camera - ((rotation_avg*lidar).colwise() + (translation_sum/iteration_counter))).array().square().colwise().sum();
       double error_temp = sqrt(eltwise_error_temp.sum()/num_points);
       log_avg_values << std::fixed << std::setprecision(8) << error_temp << "\n";

       log_avg_values.close();
    }*/



    return T;
  }


  std::vector<std::vector<Eigen::Vector3d>>
  ComputeRT::readArucoPose(std::vector<float> marker_info, int num_of_marker_in_config)
  {
    std::vector<Eigen::Matrix4d> marker_pose;

    ROS_ASSERT(marker_info.size()/7 == num_of_marker_in_config);

    int j=0;
    for(int i = 0; i < marker_info.size()/7; i++)
    {

      //std::cout << "In readArucoPose(): " << std::endl;

      Eigen::Vector3d trans, rot;
      int marker_id = marker_info[j++];
      trans(0) = marker_info[j++];
      trans(1) = marker_info[j++];
      trans(2) = marker_info[j++];
      rot(0) = marker_info[j++];
      rot(1) = marker_info[j++];
      rot(2) = marker_info[j++];

      //std::cout << "\n" << "Marker id:" << marker_id << "\n" << trans << "\n" << rot << std::endl;


      Eigen::Transform<double,3,Eigen::Affine> aa;
      aa = Eigen::AngleAxis<double>(rot.norm(), rot/rot.norm());

      Eigen::Matrix4d g;
      g.setIdentity(4,4);
      //std::cout << "Rot matrix is: \n" << aa*g << std::endl;
      g = aa*g;

      Eigen::Matrix4d T;
      T.setIdentity(4,4);
      T.topLeftCorner(3, 3) = g.topLeftCorner(3,3);//.transpose();
      T.col(3).head(3) = trans;

      marker_pose.push_back(T);

      //std::cout << "transformation matrix is: \n" << T << std::endl;
    }


    //std::vector<std::vector<std::pair<float, float> > > marker_coordinates;
    std::ifstream infile(pkg_loc + "/conf/marker_coordinates.txt");
    std::ofstream outfile(pkg_loc + "/conf/points.txt", std::ios_base::app);

    int num_of_markers;
    infile >> num_of_markers;
    std::vector<std::vector<Eigen::Vector3d>> corner_points;
    corner_points.resize(num_of_markers);

    for(int i=0; i<num_of_markers; i++)
    {
      float temp;
      std::vector<float> board;
      //std::vector<std::pair<float, float> > corner_points;
      for(int j=0; j<5; j++)
      {
        infile >> temp;
        board.push_back(temp/100.0);
      }
      float la, ba;
      la=board[4]/2+board[2];
      ba=board[4]/2+board[3];

      /*corner_points.push_back(std::make_pair(ba, 		   board[0]-la));
      corner_points.push_back(std::make_pair(ba-board[1], board[0]-la));
      corner_points.push_back(std::make_pair(ba-board[1], -la 		  ));
      corner_points.push_back(std::make_pair(ba, 		   -la 		  ));*/

      Eigen::Matrix4d points_board;
      points_board << ba, 		 0, board[0]-la, 1,
              ba-board[1], 0, board[0]-la, 1,
              ba-board[1], 0, -la, 		 1,
              ba, 		 0, -la, 		 1;

      /*std::cout << "Points in before transform: \n" << points_board << std::endl;*/

      points_board = marker_pose[i]*(points_board.transpose());

      /*std::cout << "Board number: " << i+1 << "\n";
      std::cout << "P1: " << ba << " " << board[0]-la << "\n";
      std::cout << "P2: " << ba-board[1] << " " << board[0]-la << "\n";
      std::cout << "P3: " << ba-board[1] << " " << -la << "\n";
      std::cout << "P4: " << ba << " " << -la << "\n\n";

      std::cout << "Points in camera frame: \n" << points_board << std::endl;*/

      //marker_coordinates.push_back(corner_points);

      corner_points[i].resize(4);
      for(int k=0; k < 4; k++)
      {
        outfile << points_board(0,k) << " " << points_board(1,k) << " " << points_board(2,k) <<  "\n";
        Eigen::Vector3d point(points_board(0,k),points_board(1,k),points_board(2,k));
        corner_points[i][(k+1)%4]=point;
      }

    }
    outfile.close();
    infile.close();
    return corner_points;
  }

  Eigen::Matrix4d ComputeRT::find_transformation(std::vector<float> marker_info, int num_of_marker_in_config,
                               int MAX_ITERS, Eigen::Matrix3d lidarToCamera){
    readArucoPose(marker_info, num_of_marker_in_config);
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> point_clouds = readArray();
    Eigen::Matrix4d T = calc_RT(point_clouds.first, point_clouds.second, MAX_ITERS, lidarToCamera);
    return T;
  }

  Eigen::Matrix4d ComputeRT::optimizeByCeres(Eigen::Matrix3d lidarToCamera){
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
//    double para_q[4] = {initial_rotation.x(), initial_rotation.y(), initial_rotation.z(), initial_rotation.w()};
//    double para_t[3] = {initial_translation.x(), initial_translation.y(), initial_translation.z()};
    double para_q[4] = {0,0,0,1};
    double para_t[3] = {0,0,0};
    Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
    Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

    problem.AddParameterBlock(para_q, 4, q_parameterization);
    problem.AddParameterBlock(para_t, 3);

    for(int i=0; i<all_line_points.size(); i++){
      for(int j=0; j<all_line_points[i].size(); j++){
        Eigen::Vector3d point_a = all_corner_points[i][j];
        Eigen::Vector3d point_b = all_corner_points[i][(j+1)%all_corner_points[i].size()];
//        std::cout<<"point_a="<<point_a.transpose()<<" point_b="<<point_b.transpose()<<std::endl;
        for(int k=0; k<all_line_points[i][j].size();k++){
          Eigen::Vector3d p = all_line_points[i][j][k];
//          std::cout<<"p="<<p.transpose()<<std::endl;
          ceres::CostFunction *cost_function = LidarEdgeFactor::Create(p, point_a, point_b, 1.0);
          problem.AddResidualBlock(cost_function, NULL, para_q, para_t);
//          double **para = new double *[2];
//          double *tmp_r = new double[3];
//          para[0] = para_q;
//          para[1] = para_t;
//          cost_function->Evaluate(para,tmp_r,NULL);
//          std::cout<<tmp_r[0]<<" "<<tmp_r[1]<<" "<<tmp_r[2]<<std::endl;

        }
      }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;
    options.check_gradients = false;
    options.gradient_check_relative_precision = 1e-16;
    options.function_tolerance = 1e-16;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

//    for(int i=0; i<all_line_points.size(); i++){
//      for(int j=0; j<all_line_points[i].size(); j++){
//        Eigen::Vector3d point_a = all_corner_points[i][j];
//        Eigen::Vector3d point_b = all_corner_points[i][(j+1)%all_corner_points[i].size()];
//        for(int k=0; k<all_line_points[i][j].size();k++){
//          Eigen::Vector3d p = all_line_points[i][j][k];
////          std::cout<<"point_a="<<point_a.transpose()<<" point_b="<<point_b.transpose()<<std::endl;
////          std::cout<<"p="<<p.transpose()<<std::endl;
//          ceres::CostFunction *cost_function = LidarEdgeFactor::Create(p, point_a, point_b, 1.0);
//          double **para = new double *[2];
//          double *tmp_r = new double[3];
//          para[0] = para_q;
//          para[1] = para_t;
//          cost_function->Evaluate(para,tmp_r,NULL);
//          std::cout<<tmp_r[0]<<" "<<tmp_r[1]<<" "<<tmp_r[2]<<std::endl;
//
//        }
//      }
//    }


    Eigen::Matrix3d rotation = q_last_curr.toRotationMatrix();
    Eigen::Matrix4d result;
    result.setIdentity(4,4);
    result.topLeftCorner(3, 3) = rotation;
    result.col(3).head(3) = t_last_curr;

    {
      Eigen::Matrix3d final_rotation = rotation * lidarToCamera;
      Eigen::Vector3d final_angles = final_rotation.eulerAngles(2, 1, 0);
      Eigen::Vector3d final_translation = t_last_curr;
      std::cout<<"[optimizeByCeres]lidar to camera translator"<<final_translation.transpose()<<std::endl;
      std::cout<<"[optimizeByCeres]lidar to camera ypr="<<final_angles.transpose()<<std::endl;
      Eigen::Matrix4d T;
      T.setIdentity(4,4);
      T.topLeftCorner(3, 3) = final_rotation;
      T.col(3).head(3) = final_translation;
      Eigen::Matrix4d T_lidar_camera = T.inverse();
      Eigen::Vector3d ypr_lidar_camera = T_lidar_camera.block<3,3>(0,0).eulerAngles(2, 1, 0);
      std::cout << "[optimizeByCeres]Final Camera to Lidar translation is:" << "\n" << T_lidar_camera.col(3).head(3).transpose() << "\n";
      std::cout << "[optimizeByCeres]Final Camera to Lidar ypr is:" << "\n" << ypr_lidar_camera.transpose() << "\n";
      std::cout<<"[optimizeByCeres]summary="<<summary.FullReport()<<std::endl;

      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "rslidar";//config_.parent_frame;
      transformStamped.child_frame_id = "camera_link";//config_.child_frame;
      transformStamped.transform.translation.x = T_lidar_camera.col(3).head(3)(0);
      transformStamped.transform.translation.y = T_lidar_camera.col(3).head(3)(1);
      transformStamped.transform.translation.z = T_lidar_camera.col(3).head(3)(2);
      tf2::Quaternion q;
      q.setRPY(ypr_lidar_camera(2), ypr_lidar_camera(1), ypr_lidar_camera(0));
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();
      br.sendTransform(transformStamped);
    }


    return result;
  }
  Eigen::Matrix4d ComputeRT::findTByCeres(std::vector<float> marker_info,
                                                     std::vector<std::vector<std::vector<Eigen::Vector3d>>> line_points,
                                                     int num_of_marker_in_config,
                                                     int MAX_ITERS,
                                                     Eigen::Matrix3d lidarToCamera){
    std::vector<std::vector<Eigen::Vector3d>> corner_points = readArucoPose(marker_info, num_of_marker_in_config);
    all_corner_points.insert(all_corner_points.end(),corner_points.begin(),corner_points.end());
    all_line_points.insert(all_line_points.end(),line_points.begin(),line_points.end());
//    all_corner_points = corner_points;
//    all_line_points = line_points;

    Eigen::Matrix4d T  = optimizeByCeres(lidarToCamera);

    return T;
  }
}