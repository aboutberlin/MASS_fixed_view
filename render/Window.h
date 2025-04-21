#ifndef __MASS_WINDOW_H__
#define __MASS_WINDOW_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h>
namespace py = pybind11;

namespace MASS
{
class Environment;
class Muscle;
class Window : public dart::gui::glut::Win3D
{
public:
	Window(Environment* env);
	Window(Environment* env,const std::string& nn_path);
	Window(Environment* env,const std::string& nn_path,const std::string& muscle_nn_path);

	void draw() override;
	void keyboard(unsigned char _key, int _x, int _y) override;
	void displayTimer(int _val) override;
	void SaveScreenshot();

private:
	void SetFocusing();
    void DrawEntity(const dart::dynamics::Entity* entity);

    void DrawEntity(const dart::dynamics::Entity* entity, const Eigen::Vector4d& color);
    void DrawBodyNode(const dart::dynamics::BodyNode* bn, const Eigen::Vector4d& color);
	void DrawSkeleton(const dart::dynamics::SkeletonPtr& skel);
	void DrawSkeleton(const dart::dynamics::SkeletonPtr& skel, const Eigen::Vector4d& color);

	void DrawBodyNode(const dart::dynamics::BodyNode* bn);
	void DrawShapeFrame(const dart::dynamics::ShapeFrame* sf);
	
    void DrawShapeFrame(const dart::dynamics::ShapeFrame* sf, const Eigen::Vector4d& color);
	void DrawShape(const dart::dynamics::Shape* shape,const Eigen::Vector4d& color);

	void DrawMuscles(const std::vector<Muscle*>& muscles);
	void DrawMuscles1(const std::vector<Muscle*>& muscles);
	void DrawShadow(const Eigen::Vector3d& scale, const aiScene* mesh,double y);
	void DrawAiMesh(const struct aiScene *sc, const struct aiNode* nd,const Eigen::Affine3d& M,double y);
	void DrawGround(double y);
	void Step();
	void Reset();

	Eigen::VectorXd GetActionFromNN();
	Eigen::VectorXd GetActivationFromNN(const Eigen::VectorXd& mt);

	Eigen::Vector3d exo_translation_offset = Eigen::Vector3d(0.0, 0.1, 0.0);
	Eigen::Vector3d exo_rotation_deg = Eigen::Vector3d(-3.0, -90.0, 0.0);  // X, Y, Z 旋转角



	py::scoped_interpreter guard;
	py::object mm,mns,sys_module,nn_module,muscle_nn_module;


	Environment* mEnv;
	int mViewIndex = 0;
	bool mFocus;
	bool mSimulating;
	bool mDrawOBJ;
	bool mDrawShadow;
	bool mNNLoaded;
	bool mMuscleNNLoaded;
	bool save_screenshot = true;  // 新增：是否保存图片
	int frame_counter = 0;        // 新增：帧编号

	Eigen::Affine3d mViewMatrix;
};
};


#endif