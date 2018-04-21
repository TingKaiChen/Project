# include <bshot_bits.h>

void pp_callback(const pcl::visualization::PointPickingEvent&, void*);
pcl::PointCloud<pcl::PointXYZ> cloud1_keypoints;

int main(int argc, char** argv)
{
    bshot cb;

    if ( argc != 2 )
    {
        cout<<"Usage: PointPickingTest object.pcd"<<endl;
        return 1;
    }
    //load data or scene point cloud
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], cb.cloud1);


    /// VISUALIZATION MODULE
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cb.cloud1.makeShared(), 200, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cb.cloud1.makeShared(), single_color3, "sample cloud3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud3");
    viewer->initCameraParameters ();


    while (!viewer->wasStopped ())
    {
        viewer->registerPointPickingCallback(pp_callback, (void*)&viewer);
        viewer->spin();
        // viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    // Visualize keypoints
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer2"));
    viewer2->setBackgroundColor (255, 255, 255);


    viewer2->addPointCloud<pcl::PointXYZ> (cb.cloud1.makeShared(), single_color3, "sample cloud3");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud3");
    viewer2->initCameraParameters ();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1_keypoints.makeShared(), 0, 200, 0);
    viewer2->addPointCloud<pcl::PointXYZ> (cloud1_keypoints.makeShared(), single_color1, "sample cloud1");
    viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample cloud1");
    //viewer2->addCoordinateSystem (1.0);
    viewer2->initCameraParameters ();


    while (!viewer2->wasStopped ())
    {
        viewer2->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    // Save keypoints cloud
    pcl::io::savePCDFile("test.pcd",cloud1_keypoints);
    cout<<"width:    "<<cloud1_keypoints.width<<endl;
    cout<<"height:   "<<cloud1_keypoints.height<<endl;
    cout<<"is_dense: "<<cloud1_keypoints.is_dense<<endl;
    cout<<"size:     "<<cloud1_keypoints.size()<<endl;


    return 0;
}

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
   std::cout << "Picking event active" << std::endl;
   if(event.getPointIndex()!=-1)
   {
       float x,y,z;
       int idx = event.getPointIndex();;
       event.getPoint(x,y,z);
       std::cout<<"Index: "<<idx<<"; " << x<< ";" << y<<";" << z << std::endl;
       cloud1_keypoints.push_back (pcl::PointXYZ (x,y,z));
       std::cout<<"Size of keypoints: "<<cloud1_keypoints.size()<<endl;
   }
}