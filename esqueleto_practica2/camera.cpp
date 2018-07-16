#include <camera.h>

gmtl::Rayf Camera::generateRay(float pixelX, float pixelY)
{
	gmtl::Point3f ori = { 0.0, 0.0, 0.0 };
	gmtl::Vec3f dir;

	ori = mCameraToWorld(ori);

	gmtl::Point3f dest = { pixelX, pixelY, 0.0 };

	dest = mRasterToCamera(dest);
	dest = mCameraToWorld(dest);

	dir = dest - ori;

	//gmtl::normalize(ori);
	//gmtl::normalize(dir);

	gmtl::Rayf ray(ori, dir);

    return ray;
}


gmtl::Point2ui Camera::getResolution() const
{
    return mResolution;
}

void Camera::setOutputPath(const char* filename)
{
    mOutputPath = filename;
}

const char* Camera::getOutputPath() const
{
    return mOutputPath.c_str();
}