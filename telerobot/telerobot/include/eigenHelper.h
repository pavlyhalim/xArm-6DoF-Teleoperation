#include <Eigen/Eigen>
#include <vector>
#include <math.h>

class eigenHelper
{
public:
    static void toTransformationMatrix(const Eigen::Vector3f &offset, const Eigen::Quaternionf &rotation, Eigen::Matrix4f *transformation)
    {
        *transformation << rotation.toRotationMatrix(), offset,
                           0, 0, 0, 1;
    }

    static void toVectorQuaternion(const Eigen::Matrix4f &transformation, Eigen::Vector3f *offset, Eigen::Quaternionf *rotation)
    {
        *offset << transformation(0,3),
                   transformation(1,3),
                   transformation(2,3);

        Eigen::Matrix3f rotationMatrix = transformation.topLeftCorner(3,3);
        *rotation = Eigen::Quaternionf(rotationMatrix);
    }

    static void mean(const std::vector<Eigen::Vector3f> &offsets, Eigen::Vector3f *offset)
    {
        *offset << 0,0,0;

        for(int i = 0; i<offsets.size(); i++)
        {
            Eigen::Vector3f offset_i = offsets.at(i);
            *offset = *offset + offset_i;
        }
        *offset = (*offset)/offsets.size();
    }

    static void mean(const std::vector<Eigen::Quaternionf> &rotations, Eigen::Quaternionf *rotation)
    {

        (*rotation) = rotations.at(0);

        for(int i = 1; i<rotations.size(); i++)
        {
            Eigen::Quaternionf rotation_i = rotations.at(i);
            addToMean(rotation_i,i,rotation);
        }

    }

    static void transformationBetweenTwoPlanes(const float a1, const float b1, const float c1, const float d1,\
                                               const float a2, const float b2, const float c2, const float d2,\
                                               Eigen::Matrix4f *transformation)
    {
        Eigen::Vector3f no1;
        no1 << a1,b1,c1;

        Eigen::Vector3f p1;
        p1 = d1*no1;

        Eigen::Vector3f no2;
        no2 << a2,b2,c2;

        Eigen::Vector3f p2;
        p2 = d2*no2;

        Eigen::Matrix4f translation1;
        translation1 << Eigen::Matrix3f::Identity(), p1,
                        0,0,0,1;

        Eigen::Matrix4f translation2;
        translation2 << Eigen::Matrix3f::Identity(), -p2,
                        0,0,0,1;

        Eigen::Vector3f axis = no1.cross(no2);
        axis.normalize();

        float angle = acos(no1.dot(no2));

        Eigen::AngleAxisf aa(angle,axis);

        Eigen::Matrix4f rotation;
        rotation << aa.toRotationMatrix(), Eigen::Vector3f::Zero(),
                    0,0,0,1;

        *transformation = translation2*rotation*translation1;
    }

private:

    static void addToMean(Eigen::Quaternionf &rotation, const int nrRotations, Eigen::Quaternionf *mean){

        //Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
        //q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
        if((*mean).dot(rotation)<0.0){
            rotation = rotation.inverse();
        }

        //Average the values
        (*mean).w() += rotation.w();
        (*mean).w() = (*mean).w() / (float)nrRotations;
        (*mean).x() += rotation.x();
        (*mean).x() = (*mean).x() / (float)nrRotations;
        (*mean).y() += rotation.y();
        (*mean).y() = (*mean).y() / (float)nrRotations;
        (*mean).z() += rotation.z();
        (*mean).z() = (*mean).z() / (float)nrRotations;

        //normalization
        (*mean).normalize();
    }

};
