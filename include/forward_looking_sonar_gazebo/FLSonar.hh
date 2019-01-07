#ifndef _GAZEBO_RENDERING_SONAR_HH_
#define _GAZEBO_RENDERING_SONAR_HH_

// C++ includes
#include <vector>

// Copy of modified Rock classes includes
#include "forward_looking_sonar_gazebo/Sonar.hpp"
#include "forward_looking_sonar_gazebo/Angle.hpp"

// Opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <sdf.h>

namespace gazebo {

  namespace Common {
    class Mesh;
  }

class Sonar {

public:
    /** Number of bins in a beam */
    uint32_t bin_count;

    /** Number of beams in the structure */
    uint32_t beam_count;

    /** Opening of the beam orhogonal to the device's Z direction */
    base::Angle beam_width;

    /** Opening of the beam along the device's Z direction */
    base::Angle beam_height;

    /** The speed of sound in the water in m/s */
    float speed_of_sound;

    /** Correlation between shader columns with their respective beams */
    std::vector<int> beam_cols;

    /** Store last received sonar data */
    base::samples::Sonar last_sonar;

    Sonar()
	    : bin_count(500)
	    , beam_count(0)
	    , beam_width(base::Angle::fromRad(0.0))
	    , beam_height(base::Angle::fromRad(0.0))
	    , speed_of_sound(base::samples::Sonar::getSpeedOfSoundInWater())
	    , beam_cols()
	    , last_sonar()
    {}

    Sonar(uint32_t bin_count, uint32_t beam_count, base::Angle beam_width, base::Angle beam_height)
	    : bin_count(bin_count)
	    , beam_count(beam_count)
	    , beam_width(beam_width)
	    , beam_height(beam_height)
	    , speed_of_sound(base::samples::Sonar::getSpeedOfSoundInWater())
	    , beam_cols()
	    , last_sonar()
    {}

    /**
    *  Split the shader image in beam parts. The shader is not radially spaced equally
    *  over the FOV-X degree sector, so it is needed to identify which column is contained
    *  on each beam.
    *  @param cv_image: the shader image (normal, depth and angle informations) in float
    *  @param bins: the output simulated sonar data (all beams) in float
    */
    void decodeShader(const cv::Mat& cv_image, std::vector<float>& bins);

    /**
    *  Encapsulate the simulated sonar data in the Rock's sonar datatype.
    *  @param bins: the simulated sonar data in float
    *  @param range: the maximum coveraged area in meters
    *  @return the simulated sonar in the Rock's structure
    */
    base::samples::Sonar simulateSonar(const std::vector<float>& bins, float range);

    /**
    *  Apply an additional gain in the simulated sonar data.
    *  @param bins: the simulated sonar data in float
    *  @param gain: the additional gain percent (0.0 - 1.0)
    */
    void applyAdditionalGain(std::vector<float>& bins, float gain);

private:
    /**
    *  Convert the shader image (normal and depth) in bins intensity (one beam).
    *  @param cv_image: the shader image (normal and depth informations) in float
    *  @param bins: the output simulated sonar data (one beam) in float
    */
    void convertShader(cv::Mat& cv_image, std::vector<float>& bins);

    /**
    *  Speckle is a granular 'noise' that inherently exists in and degrades the quality of
    *  underwater imaging sonars. This function adds this multiplicative noise to
    *  simulated sonar image based on gaussian distribution.
    *  @param bins: the simulated sonar data in float
    *  @param mean: the expectation of gaussian distribution in float
    *  @param stddev: the standard deviation of gaussian distribution in float
    */
    void applySpeckleNoise(std::vector<float>& bins, float mean, float stddev);

    /**
    *  Accept the input value x then returns it's sigmoid value in float.
    *  @param x: the input value in float
    *  @return the sigmoid value in float
    */
    float sigmoid(float x);

    /**
    *  Calculate the sample time period that is applied to the received sonar echo signal.
    *  @param range: the range (meters) in float
    *  @return the sampling interval
    */
    float getSamplingInterval(float range);
};
} // end namespace gpu_sonar_simulation

#endif


/////////////////////////////////////////////////////////////////////////////////////////////////

// Copyright 2018 Brazilian Intitute of Robotics"

#ifndef _GAZEBO_RENDERING_SONAR_HH_
#define _GAZEBO_RENDERING_SONAR_HH_

#include <memory>
#include <string>
#include <vector>

#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"
#include "sonar_msgs/SonarStamped.h"

#include <gazebo/physics/physics.hh>

// OpenCV includes
#include <opencv2/opencv.hpp>


namespace Ogre
{
class Material;
class Renderable;
class Pass;
class AutoParamDataSource;
class Matrix4;
class MovableObject;
}

namespace gazebo
{
namespace common
{
class Mesh;
}

namespace rendering
{

/// \addtogroup gazebo_rendering Rendering
/// \{

/// \class Sonar Sonar.hh rendering/rendering.hh
/// \brief GPU based laser distance sensor
class GZ_RENDERING_VISIBLE FLSonar
  : public Camera, public Ogre::RenderObjectListener //makes FLSonar class multi-inherit from Camera and RenderObjectListener classes
{
  /// \brief Constructor
  /// \param[in] _namePrefix Unique prefix name for the camera.
  /// \param[in] _scene Scene that will contain the camera
  /// \param[in] _autoRender Almost everyone should leave this as true.
public:
  FLSonar(const std::string &_namePrefix,
          ScenePtr _scene, const bool _autoRender = true);

  /// \brief Destructor
public:
  virtual ~FLSonar();

  // Documentation inherited
public:
  virtual void Load(sdf::ElementPtr _sdf);

  // Documentation inherited
public:
  virtual void Load();

  // Documentation inherited
public:
  virtual void Init();

  // Documentation inherited
public:
  virtual void Fini();

  /// \brief Create the texture which is used to render laser data.
  /// \param[in] _textureName Name of the new texture.
public:
  void CreateTexture(const std::string &_textureName);

  // Documentation inherited
public:
  virtual void PostRender();


  /// \internal
  /// \brief Implementation of Ogre::RenderObjectListener
public:
  virtual void notifyRenderSingleObject(Ogre::Renderable *_rend,
                                        const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
                                        const Ogre::LightList *_ll, bool _supp);

  /// \brief Get the vertical field-of-view.
  /// \return The vertical field of view of the laser sensor.
  /// \deprecated See VertFOV()
public:
  double GetVertFOV() const;

  /// \brief Get the vertical field-of-view.
  /// \return The vertical field of view of the laser sensor.
public:
  double VertFOV() const;

  /// \brief Get the horizontal field-of-view.
  /// \return The horizontal field of view of the sonar sensor.
  /// \deprecated See HorzFOV()
public:
  double GetHorzFOV() const;

  /// \brief Get the horizontal field-of-view.
  /// \return The horizontal field of view of the sonar sensor.
public:
  double HorzFOV() const;

  /// \brief Set the horizontal fov
  /// \param[in] _hfov horizontal fov
public:
  void SetHorzFOV(const double _hfov);

  /// \brief Get near clip
  /// \return near clip distance
  /// \deprecated See NearClip()
public:
  double GetNearClip() const;

  /// \brief Get near clip
  /// \return near clip distance
public:
  double NearClip() const;

  /// \brief Get far clip
  /// \return far clip distance
  /// \deprecated See FarClip()
public:
  double GetFarClip() const;

  /// \brief Get far clip
  /// \return far clip distance
public:
  double FarClip() const;

  /// \brief Get the shader output
  /// \return shader output
public:
  cv::Mat ShaderImage() const;

  /// \brief Get the sonar image on polar coordinates
  /// \return sonar image output
public:
  cv::Mat SonarImage() const;

  /// \brief Get the sonar mask for polar coordinater
  /// \return sonar mask
public:
  cv::Mat SonarMask() const;

  /// \brief Set the near clip distance
  /// \param[in] _near near clip distance
public:
  void SetNearClip(const double _near);

  /// \brief Set the far clip distance
  /// \param[in] _far far clip distance
public:
  void SetFarClip(const double _far);


  /// \brief Set the vertical fov
  /// \param[in] _vfov vertical fov
public:
  void SetVertFOV(const double _vfov);

  // Documentation inherited.
public:
  virtual void RenderImpl();


  /// \brief Update a render target.
  /// \param[in, out] _target Render target to update (render).
  /// \param[in, out] _material Material used during render.
  /// \param[in] _cam Camera to render from.
  /// \param[in] _updateTex True to update the textures in the material
private:
  void UpdateRenderTarget(Ogre::RenderTarget *_target,
                          Ogre::Material *_material,
                          Ogre::Camera *_cam,
                          const bool _updateTex = false);


  /**
   * @brief Print the results to a file
   *
   * @param _width
   * @param _height
   * @param _inTex
   */
protected:
  void ImageTextureToCV(float _width, int _height, Ogre::Texture* _inTex);

  /**
   * @brief Documentation Iherited
   *
   * @param _type
   * @return true
   * @return false
   */
public:
  virtual bool SetProjectionType(const std::string &_type);

  /**
   * @brief Pre render step
   *
   * @param _pose Set position of the camera
   */
public:
  void PreRender(const math::Pose &_pose);

  /**
   * @brief Get the Sonar Image to cartesian cv::Mat
   *
   */
public:
  void GetSonarImage();

  /**
   * @brief Get the Ros sonar msg
   *
   */
public:
  sonar_msgs::SonarStamped SonarRosMsg(const physics::WorldPtr _world);

  /**
   * @brief Update the data for the sonar
   *
   */
public:
  void UpdateData();

  /**
   * @brief Cv mat to sonar bin data
   *
   * @param _accumData vector with all image data
   */
  void CvToSonarBin(std::vector<float> &_accumData);

  /**
   * @brief Create transfer table from cartesian to polar
   *
   * @param _transfer Transfer vector that will be Generated
   */
protected:
  void GenerateTransferTable(std::vector<int> &_transfer);

  /**
   * @brief Transfer the sonar bin data to cv::Mat sonarImage using transfer matrix
   *
   * @param _accumData Vector with sonar bins data
   * @param _transfer Vector with tranfer function cartesian to polar
   */
protected:
  void TransferTableToSonar(const std::vector<float> &_accumData, const std::vector<int> &_transfer);

  /**
   * @brief
   *
   * @param _texture
   * @param _image
   * @param _width
   * @param _height
   */
protected:
  void PixelBoxTextureToCV(Ogre::Texture *_texture, cv::Mat &_image, int _width, int _height);

  /**
   * @brief Get image width
   *
   * @return int Image width
   */
public:
  int ImageWidth();

  /**
   * @brief Get image height
   *
   * @return int Image height
   */
public:
  int ImageHeight();

  /**
   * @brief Get bin count
   *
   * @return int Bin count
   */
public:
  int BinCount();

  /**
   * @brief Get beam count
   *
   * @return int Beam count
   */
public:
  int BeamCount();


  /**
   * @brief Set the Bin Count object
   *
   * @param _value
   */
public:
  void SetBinCount(const int &_value);

  /**
   * @brief Set the Beam Count object
   *
   * @param _value
   */
public:
  void SetBeamCount(const int &_value);

  /**
   * @brief Set the Image Width object
   *
   * @param _value
   */
public:
  void SetImageWidth(const int &_value);

  /**
   * @brief Set the Image Height object
   *
   * @param _value
   */
public:
  void SetImageHeight(const int &_value);


  /**
   * @brief Simple sigmoid function for bin intensity calculation
   *
   * @param x Input of sigmoid function
   * @return float Sigmoid value
   */
protected:
  float Sigmoid(float x);

  /// \brief Vertical field-of-view.
protected:
  double vfov;

/// \brief Horizontal field-of-view.
protected:
  double hfov;


  /// \brief Near clip plane.
protected:
  double nearClip;

  /// \brief Far clip plane.
protected:
  double farClip;

  //// \brief Camera Texture for rendering
public:
  Ogre::Texture* camTexture;

  //// \brief Camera Material
public:
  Ogre::Material *camMaterial;

  //// \brief Camera Target
public:
  Ogre::RenderTarget *camTarget;

  //// \brief Sonar beams depth data
public:
  std::vector<int> sonarBinsDepth;

  //// \brief Number of bins
protected:
  int binCount;

  //// \brief Image width for texture
protected:
  int imageWidth;

  //// \brief Image height for texture
protected:
  int imageHeight;

  //// \brief Bins vector
public:
  std::vector<float> bins;

  //// \brief Image mask for polar image output
protected:
  cv::Mat sonarImageMask;

  //// \brief Image of the pure sonar image in cartesian coordinates
public:
  cv::Mat sonarImage;

  //// \brief Raw image with texture data
protected:
  cv::Mat rawImage;

  //// \brief Number of beams
protected:
  int beamCount;

  //// \brief Data from sensor
protected:
  std::vector<float> accumData;

/// \brief Flag to check if the message was updated.
private:
  bool bUpdated;

  //// \brief Sonar image from ogre
  Ogre::Image imgSonar;

  /**
   * @brief Debug Functions
   *
   */

  /**
   * @brief Print the vector to a matrix file with size beamCount per binCount
   *
   * @tparam T Content type of the matrix (float or int)
   * @param _filename FileName output
   * @param _matrix Input vector with matrix data
   */

private:
  template <typename T> void DebugPrintMatrixToFile(const std::string &_filename, const std::vector<T> &_matrix);

  /**
   * @brief Print the image to a matrix file with size row x col
   *
   * @param _filename Filename output
   * @param _image Name of the output file
   */
private:
  void DebugPrintImageToFile(const std::string &_filename, const cv::Mat &_image);

  /**
   * @brief Print the texture into a png file named "MyCamTest.png"
   *
   * @param _texture
   */
private:
  void DebugPrintTexture(Ogre::Texture *_texture);

  /**
   * @brief Print a channel of Image file to a matrix colxrow
   *
   * @param _filename Name of the output file
   * @param _image Image to be written
   * @param _channel Channel selected
   */
private:
  void DebugPrintImageChannelToFile(const std::string &_filename, const cv::Mat &_image, const int &_channel);
};
}  // namespace rendering
}  // namespace gazebo
#endif
