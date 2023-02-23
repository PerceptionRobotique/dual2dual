/*!
 \file dual2dual.cpp
 \brief Dual fisheye to dual fisheye mapping (RGB image format), exploiting PeR core and io modules
 * example of command line :

  ./dual2dual ../data/RicohThetaS_calib_withTrans.xml ../media/ 0 9 1 ../data/maskFull.png ../media/poses.txt 1

 \param xmlFic the dualfisheye camera calibration xml file (the data directory stores an example of a dualfisheye camera xml file)
 \param imagesDir directory where dualfisheye images to read (with 6 digits before the extension) are and where the output equirectangular images will be written (with characters 'e_' before the 6 digits)
 \param iFirst the number of the first image to transform
 \param iLast the number of the last image to transform
 \param iStep the increment to the next image to transform
 \param maskFic the dualfisheye image mask to discard useless areas (png file: a black pixel is not to be transformed)
 \param posesFic a text file of one 3D pose per image (one row - one image) stored as the 3 elements of the translation vector followed by the 3 elements of the axis-angle vector representation of the rotation
 \param iInvertPose a flag to let the program knows if poses of posesFic must be applied to images as they are or inversed

 *
 \author Guillaume CARON, Antoine ANDRÉ
 \version 0.1
 \date Feb 2023
*/

#include <iostream>
#include <iomanip>

#include <per/prcommon.h>
#include <per/prOmni.h>
#include <per/prEquirectangular.h>
#include <per/prStereoModel.h>
#include <per/prStereoModelXML.h>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

// Uncomment this define for verbose content
// #define VERBOSE

/*!
 * \fn main()
 * \brief Main function of the dual fisheye to equirangular spherical image transformation
 *
 * 1. Loading a divergent stereovision system made of two fisheye cameras considering the Barreto's model from an XML file got from the MV calibration software
 *
 *
 */
int main(int argc, char **argv)
{
    // Loading the twinfisheye intrinsic parameters
    if (argc < 2)
    {
#ifdef VERBOSE
        std::cout << "no XML stereo rig file given" << std::endl;
#endif
        return -1;
    }

    // Create an empty rig
    prStereoModel stereoCam(2);

    // Load the stereo rig parameters from the XML file
    {
        prStereoModelXML fromFile(argv[1]);

        fromFile >> stereoCam;
    }

    vpHomogeneousMatrix c2Rc1 = stereoCam.sjMr[1];

#ifdef VERBOSE
    std::cout << "Loading the XML file to an empty rig..." << std::endl;

    // If a sensor is loaded, print its parameters
    if (stereoCam.get_nbsens() >= 1)
    {
        std::cout << "the stereo rig is made of a " << ((prCameraModel *)stereoCam.sen[0])->getName() << " camera of intrinsic parameters alpha_u = " << ((prCameraModel *)stereoCam.sen[0])->getau() << " ; alpha_v = " << ((prCameraModel *)stereoCam.sen[0])->getav() << " ; u_0 = " << ((prCameraModel *)stereoCam.sen[0])->getu0() << " ; v_0 = " << ((prCameraModel *)stereoCam.sen[0])->getv0();
        if (((prCameraModel *)stereoCam.sen[0])->getType() == Omni)
            std::cout << " ; xi = " << ((prOmni *)stereoCam.sen[0])->getXi();
        std::cout << "..." << std::endl;
    }

    // If a second sensor is loaded, print its parameters and its pose relatively to the first camera
    if (stereoCam.get_nbsens() >= 2)
    {
        std::cout << "... and a " << ((prCameraModel *)stereoCam.sen[1])->getName() << " camera of intrinsic parameters alpha_u = " << ((prCameraModel *)stereoCam.sen[1])->getau() << " ; alpha_v = " << ((prCameraModel *)stereoCam.sen[1])->getav() << " ; u_0 = " << ((prCameraModel *)stereoCam.sen[1])->getu0() << " ; v_0 = " << ((prCameraModel *)stereoCam.sen[1])->getv0();
        if (((prCameraModel *)stereoCam.sen[1])->getType() == Omni)
            std::cout << " ; xi = " << ((prOmni *)stereoCam.sen[1])->getXi();
        std::cout << "..." << std::endl;

        std::cout << "...at camera pose cpMco = " << std::endl
                  << stereoCam.sjMr[1] << std::endl
                  << " relative to the first camera." << std::endl;
    }
#endif

    // Loading the reference image with respect to which the cost function will be computed
    if (argc < 3)
    {
#ifdef VERBOSE
        std::cout << "no image files directory path given" << std::endl;
#endif
        return -4;
    }

    // Get filename thanks to boost
    char myFilter[1024];
    char *chemin = (char *)argv[2];
    char ext[] = "png";

    if (argc < 4)
    {
#ifdef VERBOSE
        std::cout << "no initial image file number given" << std::endl;
#endif
        return -6;
    }
    unsigned int i0 = atoi(argv[3]); // 1;//0;

    if (argc < 5)
    {
#ifdef VERBOSE
        std::cout << "no image files count given" << std::endl;
#endif
        return -7;
    }
    unsigned int i360 = atoi(argv[4]);

    if (argc < 6)
    {
#ifdef VERBOSE
        std::cout << "no image sequence step given" << std::endl;
#endif
        return -8;
    }
    unsigned int iStep = atoi(argv[5]);

    // lecture de l'image "masque"
    // Chargement du masque
    // vpImage<unsigned char> Mask;
    cv::Mat imgMask;
    if (argc < 7)
    {
#ifdef VERBOSE
        std::cout << "no mask image given" << std::endl;
#endif
    }
    else
    {
        try
        {
            imgMask = cv::imread(argv[6]);
        }
        catch (vpException e)
        {
            std::cout << "unable to load mask file" << std::endl;
            return -8;
        }
    }

    // fichier avec les poses initiales r_0
    bool ficInit = false;
    std::vector<vpPoseVector> v_pv_init;
    if (argc < 8)
    {
#ifdef VERBOSE
        std::cout << "no initial poses file given" << std::endl;
#endif
        // return -9;
    }
    else
    {
        ficInit = true;

        std::ifstream ficPosesInit(argv[7]);
        if (!ficPosesInit.good())
        {
#ifdef VERBOSE
            std::cout << "poses file " << argv[7] << " not existing" << std::endl;
#endif
            return -9;
        }
        vpPoseVector r;
        while (!ficPosesInit.eof())
        {
            ficPosesInit >> r[0] >> r[1] >> r[2] >> r[3] >> r[4] >> r[5];
            v_pv_init.push_back(r);
        }
        ficPosesInit.close();
    }

    // direct or inverse pose
    unsigned int inversePose = 0;
    if (argc < 9)
    {
#ifdef VERBOSE
        std::cout << "no stabilisation parameter given" << std::endl;
#endif
        // return -9;
    }
    else
        inversePose = atoi(argv[8]);

    // Pour chaque image du dataset
    int nbPass = 0;
    bool clickOut = false;
    unsigned int imNum = i0;
    double temps;
    std::vector<double> v_temps;
    v_temps.reserve((i360 - i0) / iStep);

    vpHomogeneousMatrix cMc0;

    cv::Mat imgIn, imgOut;

    boost::filesystem::path dir(chemin);
    boost::regex my_filter;
    std::string name;
    std::ostringstream s;
    std::string filename;

    prPointFeature P1, P2;
    double Xs, Ys, Zs, u, v, dv, du, unmdv, unmdu;
    unsigned int icam, imWidth, imHeight;
    unsigned int i, j;
    cv::Vec3b pixColor;

    while (imNum <= i360)
    {

        std::chrono::steady_clock::time_point temps = std::chrono::steady_clock::now();
        std::cout << "num request image : " << nbPass << std::endl;

        // load source image
        sprintf(myFilter, "%06d.*\\.%s", imNum, ext);

        my_filter.set_expression(myFilter);

        for (boost::filesystem::directory_iterator iter(dir), end; iter != end; ++iter)
        {
            name = iter->path().leaf().string();
            if (boost::regex_match(name, my_filter))
            {
                std::cout << iter->path().string() << " loaded" << std::endl;
                imgIn = cv::imread(iter->path().string());
                break;
            }
        }

        if (nbPass == 0)
        {
            imWidth = imgIn.cols;
            imHeight = imgIn.rows;
            imgOut = cv::Mat(imHeight, imWidth, imgIn.type());

            if (imgMask.cols == 0)
                imgMask = cv::Mat(imgIn.rows, imgIn.cols, imgIn.type());
        }

        if (v_pv_init.size() > nbPass)
        {
            vpRxyzVector rotVecDesired(v_pv_init[nbPass][3], v_pv_init[nbPass][4], v_pv_init[nbPass][5]);
            vpRotationMatrix rotMatDesired(rotVecDesired);
            // vpRotationMatrix rotMatTotAlign = (rotMatDesired.inverse() * rotMatBase);
            // dMcDesired.buildFrom(nullTrans, rotMatTotAlign);
            cMc0.buildFrom(vpTranslationVector(0, 0, 0), rotMatDesired);
            if (inversePose)
                cMc0 = cMc0.inverse();
        }
        else
            cMc0.eye();

        bool projOk1, projOk2;

        for (unsigned int v_er = 0; v_er < imgOut.rows; v_er++)
        {
            for (unsigned int u_er = 0; u_er < imgOut.cols; u_er++)
            {
                u = 0;
                v = 0;

                P1.set_u(u_er);
                P1.set_v(v_er);
                P2.set_u(u_er);
                P2.set_v(v_er);

                ((prOmni *)(stereoCam.sen[0]))->pixelMeterConversion(P1);
                projOk1 = ((prOmni *)(stereoCam.sen[0]))->projectImageSphere(P1, Xs, Ys, Zs);
                ((prOmni *)(stereoCam.sen[1]))->pixelMeterConversion(P2);
                projOk2 = ((prOmni *)(stereoCam.sen[1]))->projectImageSphere(P2, Xs, Ys, Zs);

                if (projOk2 && Zs > 0)
                {

                    P2.set_oX(Xs);
                    P2.set_oY(Ys);
                    P2.set_oZ(Zs);
                    P2.changeFrame(cMc0 * c2Rc1);

                    if (P2.get_Z() > 0)
                    {
                        icam = 0;
                    }
                    else
                    {
                        icam = 1;
                        P2.sX = P2.sX.changeFrame(c2Rc1);
                    }
                    ((prOmni *)(stereoCam.sen[icam]))->project3DImage(P2);
                    ((prOmni *)(stereoCam.sen[icam]))->meterPixelConversion(P2);
                    u = P2.get_u();
                    v = P2.get_v();
                }

                if (projOk1 && Zs > 0)
                {

                    P1.set_oX(Xs);
                    P1.set_oY(Ys);
                    P1.set_oZ(Zs);
                    P1.changeFrame(cMc0);

                    if (P1.get_Z() > 0)
                    {
                        icam = 0;
                    }
                    else
                    {
                        icam = 1;
                        P1.sX = P1.sX.changeFrame(c2Rc1);
                    }
                    ((prOmni *)(stereoCam.sen[icam]))->project3DImage(P1);
                    ((prOmni *)(stereoCam.sen[icam]))->meterPixelConversion(P1);
                    u = P1.get_u();
                    v = P1.get_v();
                }

                if ((u >= 0) && (v >= 0) && (u < (imWidth - 1)) && (v < (imHeight - 1)))
                {
                    pixColor = cv::Vec3b(0, 0, 0);

                    i = (int)v;
                    dv = v - i;
                    unmdv = 1.0 - dv;
                    j = (int)u;
                    du = u - j;
                    unmdu = 1.0 - du;

                    if (imgMask.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0))
                    {
                        pixColor = pixColor + imgIn.at<cv::Vec3b>(i, j) * unmdv * unmdu;
                    }

                    if (imgMask.at<cv::Vec3b>(i + 1, j) != cv::Vec3b(0, 0, 0))
                    {
                        pixColor = pixColor + imgIn.at<cv::Vec3b>(i + 1, j) * dv * unmdu;
                    }

                    if (imgMask.at<cv::Vec3b>(i, j + 1) != cv::Vec3b(0, 0, 0))
                    {
                        pixColor = pixColor + imgIn.at<cv::Vec3b>(i, j + 1) * unmdv * du;
                    }

                    if (imgMask.at<cv::Vec3b>(i + 1, j + 1) != cv::Vec3b(0, 0, 0))
                    {
                        pixColor = pixColor + imgIn.at<cv::Vec3b>(i + 1, j + 1) * dv * du;
                    }

                    imgOut.at<cv::Vec3b>(v_er, u_er) = pixColor;
                }
            }
        }

        v_temps.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - temps).count());
        std::cout << "Pass " << nbPass << " time : " << v_temps[nbPass] << " ms" << std::endl;

        cv::imshow("out dual", imgOut);
        cv::waitKey(1);

        // save the equirectangular image
        s.str("");
        s.setf(std::ios::right, std::ios::adjustfield);
        s << chemin << "/dual2dual/dual2dual_convert_" << std::setfill('0') << std::setw(6) << imNum << "." << ext;
        filename = s.str();
        cv::imwrite(filename, imgOut);

        imNum += iStep;
        nbPass++;
    }

    // save times list to file
    // s.str("");
    // s.setf(std::ios::right, std::ios::adjustfield);
    // s << chemin << "/e_time_" << i0 << "_" << i360 << ".txt";
    // filename = s.str();
    // std::ofstream ficTime(filename.c_str());
    // std::vector<double>::iterator it_time = v_temps.begin();
    // for (; it_time != v_temps.end(); it_time++)
    // {
    //     ficTime << *it_time << std::endl;
    // }
    // ficTime.close();

    return 0;
}
