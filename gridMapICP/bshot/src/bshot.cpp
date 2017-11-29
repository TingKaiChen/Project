# include <bshot_bits.h>

int main(int argc, char** argv)
{
    bshot cb;

    if ( argc != 3 )
    {
        cout<<"Usage: bshot target.pcd source.pcd"<<endl;
        return 1;
    }
    //load data or scene point cloud
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], cb.cloud2);
    /// load model
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], cb.cloud1);
    //pcl::io::loadPCDFile<pcl::PointXYZ>("../sample_pcd/PeterRabbit015_0.pcd", cb.cloud1);
    //pcl::io::loadPCDFile<pcl::PointXYZ>("../sample_pcd/mario000_0.pcd", cb.cloud1);

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    cb.calculate_normals (25);
    cb.calculate_voxel_grid_keypoints (1.0);
    // The support size can be varied and number of matches and computational time may change accordingly
    cb.calculate_SHOT (15);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "Time taken for SHOT Creation : " << time_span.count() << std::endl;

    clock_t start0, end0;
    double cpu_time_used0;
    start0 = clock();

    cb.compute_bshot();

    end0 = clock();
    cpu_time_used0 = ((double) (end0 - start0)) / CLOCKS_PER_SEC;
    std::cout << "Time taken for Binary SHOT Creation : " << (double)cpu_time_used0 << std::endl;

    clock_t start2, end2;
    double cpu_time_used2;
    start2 = clock();

    pcl::Correspondences corresp;

    int *dist = new int[std::max(cb.cloud1_bshot.size(), cb.cloud2_bshot.size())];
    int *left_nn = new int[cb.cloud1_bshot.size()];
    int *right_nn = new int[cb.cloud2_bshot.size()];

    int min_ix;
    for (int i=0; i<(int)cb.cloud1_bshot.size(); ++i)
    {
        for (int k=0; k<(int)cb.cloud2_bshot.size(); ++k)
        {
            dist[k] = (int)( cb.cloud1_bshot[i].bits ^ cb.cloud2_bshot[k].bits).count();
        }
        minVect(dist, (int)cb.cloud2_bshot.size(), &min_ix);
        left_nn[i] = min_ix;
    }
    for (int i=0; i<(int)cb.cloud2_bshot.size(); ++i)
    {
        for (int k=0; k<(int)cb.cloud1_bshot.size(); ++k)
            dist[k] = (int)(cb.cloud2_bshot[i].bits ^ cb.cloud1_bshot[k].bits).count();
        minVect(dist, (int)cb.cloud1_bshot.size(), &min_ix);
        right_nn[i] = min_ix;
    }

    for (int i=0; i<(int)cb.cloud1_bshot.size(); ++i)
        if (right_nn[left_nn[i]] == i)
        {
            pcl::Correspondence corr;
            corr.index_query = i;
            corr.index_match = left_nn[i];
            corresp.push_back(corr);
        }

    delete [] dist;
    delete [] left_nn;
    delete [] right_nn;

    end2 = clock();
    cpu_time_used2 = ((double) (end2 - start2)) / CLOCKS_PER_SEC;
    std::cout << "Time taken for NN Matching : " << (double)cpu_time_used2 << std::endl;

    clock_t start1, end1;
    double cpu_time_used1;
    start1 = clock();

    /// RANSAC BASED Correspondence Rejection
    pcl::CorrespondencesConstPtr correspond = boost::make_shared< pcl::Correspondences >(corresp);

    pcl::Correspondences corr;
    pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZ > Ransac_based_Rejection;
    Ransac_based_Rejection.setInputSource(cb.cloud1_keypoints.makeShared());
    Ransac_based_Rejection.setInputTarget(cb.cloud2_keypoints.makeShared());
    double sac_threshold = 0.05;// default PCL value..can be changed and may slightly affect the number of correspondences
    Ransac_based_Rejection.setInlierThreshold(sac_threshold);
    Ransac_based_Rejection.setInputCorrespondences(correspond);
    Ransac_based_Rejection.getCorrespondences(corr);

    Eigen::Matrix4f mat = Ransac_based_Rejection.getBestTransformation();
    cout << "Mat : \n" << mat << endl;

    end1 = clock();
    cpu_time_used1 = ((double) (end1 - start1)) / CLOCKS_PER_SEC;
    std::cout << "Time taken for Outlier Removal via RANSAC : " << (double)cpu_time_used1 << std::endl;

    cout << "No. of B-SHOT Matches : " << corr.size() << endl;


    /// VISUALIZATION MODULE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cb.cloud1_keypoints.makeShared(), 200, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cb.cloud1_keypoints.makeShared(), single_color1, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample cloud1");
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    Eigen::Matrix4f t;
    t<<1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    //cloudNext is my target cloud
    pcl::transformPointCloud(cb.cloud2_keypoints,cb.cloud2_keypoints,t);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cb.cloud2_keypoints.makeShared(), 0, 0, 150);
    viewer->addPointCloud<pcl::PointXYZ> (cb.cloud2_keypoints.makeShared(), single_color2, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample cloud2");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cb.cloud1.makeShared(), 0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cb.cloud1.makeShared(), single_color3, "sample cloud3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud3");

    //cloudNext is my target cloud
    pcl::transformPointCloud(cb.cloud2,cb.cloud2,t);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color4(cb.cloud2.makeShared(), 0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cb.cloud2.makeShared(), single_color4, "sample cloud4");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud4");

    viewer->addCorrespondences<pcl::PointXYZ>(cb.cloud1_keypoints.makeShared(), cb.cloud2_keypoints.makeShared(), corr/*corresp*/, "correspondences");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}

