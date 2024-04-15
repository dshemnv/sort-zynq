/**
 * @file acqsys.hpp
 * @author Denis Shemonaev
 * @brief Acquisition system module.
 * @version 0.1
 * @date 2024-01-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef ACQSYS_H
#define ACQSYS_H
#include "utils.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

/**
 * @brief Acquisition system interface, should be overloaded to implement new
 * acquisition systems.
 *
 */
class AqSys {
  protected:
    int currentFrameIdx;  //!< Current frame number.
    cv::Mat currentFrame; //!< Current frame buffer.

  public:
    virtual bool eof()      = 0; //!< Signals end of file.
    virtual void getFrame() = 0; //!< Grabs next frame and stores it
                                 //!< in #currentFrame.
    virtual const cv::Mat &getCurrentFrame() = 0; //!< Returns the buffered
                                                  //!< frame #currentFrame
    virtual ~AqSys(){};
    virtual int index() = 0; //!< Returs current index, i.e. #currentFrameIdx.
};

/**
 * @brief Implements camera input, based on the device id (/dev/videoX) on Linux
 *
 */
class AqSysCam : public AqSys {
  private:
    cv::VideoCapture stream;

  public:
    /**
     * @brief Construct a new AcqSys Cam object
     *
     * @param devId device identifier (/dev/videoX)
     */
    AqSysCam(int devId);
    ~AqSysCam();
    /**
     * @brief Get the next Frame object from @a stream and put it in a buffer @a
     * currentFrame. Also advances the @a currentFrameIdx.
     *
     */
    void getFrame();
    /**
     * @brief Return the @a currentFrame object.
     *
     * @return const cv::Mat&
     */
    const cv::Mat &getCurrentFrame();
    /**
     * @brief Signals end of file, i.e, @a stream is closed.
     *
     * @return true
     * @return false
     */
    bool eof();
    /**
     * @brief Returns the current frame index @a currentFrameIdx.
     *
     * @return int
     */
    int index();
};
/**
 * @brief Implements an input from a folder containing images.
 * @note Used for MOT Challenge folders.
 *
 */
class AqSysFiles : public AqSys {
  protected:
    std::vector<std::string> imgPaths; /**< List of image paths*/
    std::string name;                  /**< Name of the AcqSys*/

  public:
    /**
     * @brief Construct a new Acq Sys Files object
     *
     * @param folder Folder containing the images
     */
    AqSysFiles(const std::string &folder);
    AqSysFiles();
    ~AqSysFiles();
    /**
     * @brief Add an image to the list #imgPaths.
     *
     * @param path Path to image.
     */
    void addImgFile(const std::string &path);
    /**
     * @brief Signals end of file. i.e. no more images in the folder.
     *
     * @return true
     * @return false
     */
    bool eof();
    /**
     * @brief Get the next Frame object
     *
     */
    void getFrame();
    const cv::Mat &getCurrentFrame();
    const std::string &getName();
    int index();
    /**
     * @brief Returns a reference to the #currentFrameIdx
     * @return int&
     */
    int &frameCounter();
    /**
     * @brief Returns the size of #imgPaths.
     *
     * @return int
     */
    int size();
};

/**
 * @brief Implements MOT Challenge folder as an input.
 *
 */
class AqSysMOT : public AqSysFiles {
  private:
    std::string path;

  public:
    AqSysMOT(const std::string &folder);
    ~AqSysMOT();
    void load();
};

#endif