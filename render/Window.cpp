#include "Window.h"
#include "Environment.h"
#include "Character.h"
#include "BVH.h"
#include "Muscle.h"
#include <iostream>
using namespace MASS;
using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;

Window::
Window(Environment* env)
	:mEnv(env),mViewIndex(2), mFocus(true),mSimulating(false),mDrawOBJ(false),mDrawShadow(false),mMuscleNNLoaded(false)
{
	mBackground[0] = 1.0;
	mBackground[1] = 1.0;
	mBackground[2] = 1.0;
	mBackground[3] = 1.0;
	SetFocusing();
	mZoom = 0.25;	
	mFocus = false;
	mNNLoaded = false;

	mm = py::module::import("__main__");
	mns = mm.attr("__dict__");
	sys_module = py::module::import("sys");
	
	py::str module_dir = (std::string(MASS_ROOT_DIR)+"/python").c_str();
	sys_module.attr("path").attr("insert")(1, module_dir);
	py::exec("import torch",mns);
	py::exec("import torch.nn as nn",mns);
	py::exec("import torch.optim as optim",mns);
	py::exec("import torch.nn.functional as F",mns);
	py::exec("import torchvision.transforms as T",mns);
	py::exec("import numpy as np",mns);
	py::exec("from Model import *",mns);
}
Window::
Window(Environment* env,const std::string& nn_path)
	:Window(env)
{
	mNNLoaded = true;

	py::str str = ("num_state = "+std::to_string(mEnv->GetNumState())).c_str();
	py::exec(str,mns);
	str = ("num_action = "+std::to_string(mEnv->GetNumAction())).c_str();
	py::exec(str,mns);

	nn_module = py::eval("SimulationNN(num_state,num_action)",mns);

	py::object load = nn_module.attr("load");
	load(nn_path);
}
Window::
Window(Environment* env,const std::string& nn_path,const std::string& muscle_nn_path)
	:Window(env,nn_path)
{
	mMuscleNNLoaded = true;

	py::str str = ("num_total_muscle_related_dofs = "+std::to_string(mEnv->GetNumTotalRelatedDofs())).c_str();
	py::exec(str,mns);
	str = ("num_actions = "+std::to_string(mEnv->GetNumAction())).c_str();
	py::exec(str,mns);
	str = ("num_muscles = "+std::to_string(mEnv->GetCharacter()->GetMuscles().size())).c_str();
	py::exec(str,mns);

	muscle_nn_module = py::eval("MuscleNN(num_total_muscle_related_dofs,num_actions,num_muscles)",mns);

	py::object load = muscle_nn_module.attr("load");
	load(muscle_nn_path);
}
void
Window::
draw()
{	
	glEnable(GL_MULTISAMPLE);  // 启用抗锯齿（默认是 4x）
	SetFocusing();
	GLfloat matrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	Eigen::Matrix3d A;
	Eigen::Vector3d b;
	A<<matrix[0],matrix[4],matrix[8],
	matrix[1],matrix[5],matrix[9],
	matrix[2],matrix[6],matrix[10];
	b<<matrix[12],matrix[13],matrix[14];
	mViewMatrix.linear() = A;
	mViewMatrix.translation() = b;

	auto ground = mEnv->GetGround();
	float y = ground->getBodyNode(0)->getTransform().translation()[1] + dynamic_cast<const BoxShape*>(ground->getBodyNode(0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
	
	// DrawGround(y);
	DrawMuscles1(mEnv->GetCharacter()->GetMuscles());
	// 奶白色人体
	DrawSkeleton(mEnv->GetCharacter()->GetSkeleton(), Eigen::Vector4d(0.98, 0.94, 0.88, 1.0));

	// 偏蓝色的外骨骼
	// if (mEnv->UseExo()) {
	// 	DrawSkeleton(mEnv->GetCharacter()->GetExoSkeleton(), Eigen::Vector4d(0.66, 0.69, 0.70, 1.0));
	// }
	// Eigen::Quaterniond q = mTrackBall.getCurrQuat();
	// q.x() = 0.0;
	// q.z() = 0.0;
	// q.normalize();
	// mTrackBall.setQuaternion(q);
	// === 截图保存功能 ===
	// static int frame_counter = 0;
	// static int screenshot_counter = 0;
	// bool save_screenshot = true;

	// if (save_screenshot && (frame_counter % 20 == 0) && screenshot_counter < 1000) {
	// 	GLint viewport[4];
	// 	glGetIntegerv(GL_VIEWPORT, viewport);
	// 	std::cout << "Viewport size: " << viewport[2] << "x" << viewport[3] << std::endl;		
	// 	int width = viewport[2];
	// 	int height = viewport[3];		
	// 	unsigned char* pixels = new unsigned char[3 * width * height];

	// 	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	// 	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	// 	// 翻转图像
	// 	for (int j = 0; j < height / 2; ++j) {
	// 		for (int i = 0; i < width * 3; ++i) {
	// 			std::swap(pixels[j * width * 3 + i], pixels[(height - j - 1) * width * 3 + i]);
	// 		}
	// 	}

	// 	char filename[128];
	// 	sprintf(filename, "screenshot_%03d.ppm", screenshot_counter++);
	// 	FILE* fp = fopen(filename, "wb");
	// 	fprintf(fp, "P6\n%d %d\n255\n", width, height);
	// 	fwrite(pixels, 1, width * height * 3, fp);
	// 	fclose(fp);
	// 	delete[] pixels;
	// }

	// frame_counter++;  // 每次 draw 调用后都自增


}
void
Window::
keyboard(unsigned char _key, int _x, int _y)
{
	switch (_key)
	{
	case 's': this->Step();break;
	// case 'f': mFocus = !mFocus;break;
	case 'r': this->Reset();break;
	case ' ': mSimulating = !mSimulating;break;
	case 'o': mDrawOBJ = !mDrawOBJ;break;
	case 'p': SaveScreenshot(); break;
	case 'v': mViewIndex = (mViewIndex + 1) % 4; 
    SetFocusing();                     
    break;
	
	case 'z':
	exo_translation_offset[0] -= 0.01;
	mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

	break;
	case 'x':
		exo_translation_offset[0] += 0.01;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'c':
		exo_translation_offset[2] -= 0.01;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'b':
		exo_translation_offset[2] += 0.01;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'n':
		exo_translation_offset[1] += 0.01;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'm':
		exo_translation_offset[1] -= 0.01;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'l':
		exo_rotation_deg[1] -= 1.0;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'k':
		exo_rotation_deg[1] += 1.0;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'j':
		exo_rotation_deg[0] -= 1.0;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'h':
		exo_rotation_deg[0] += 1.0;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'g':
		exo_rotation_deg[2] -= 1.0;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;
	case 'f':
		exo_rotation_deg[2] += 1.0;
		mEnv->GetCharacter()->AlignExoToHuman(exo_translation_offset, exo_rotation_deg);

		break;

    
	case 27 : exit(0);break;
	default:
		Win3D::keyboard(_key,_x,_y);break;
	}

}
void
Window::
displayTimer(int _val)
{
	if(mSimulating)
		Step();
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}
void
Window::
Step()
{	
	int num = mEnv->GetSimulationHz()/mEnv->GetControlHz();
	Eigen::VectorXd action;
	if(mNNLoaded)
		action = GetActionFromNN();
	else
		action = Eigen::VectorXd::Zero(mEnv->GetNumAction());
	mEnv->SetAction(action);

	if(mEnv->GetUseMuscle())
	{
		int inference_per_sim = 2;
		for(int i=0;i<num;i+=inference_per_sim){
			Eigen::VectorXd mt = mEnv->GetMuscleTorques();
			mEnv->SetActivationLevels(GetActivationFromNN(mt));
			for(int j=0;j<inference_per_sim;j++)
				mEnv->Step();
		}	
	}
	else
	{
		for(int i=0;i<num;i++)
			mEnv->Step();	
	}
	
}
void
Window::
Reset()
{
	mEnv->Reset();
}
// void
// Window::
// SetFocusing()
// {
// 	if(mFocus)
// 	{
// 		mTrans = -mEnv->GetWorld()->getSkeleton("Human")->getRootBodyNode()->getCOM();
// 		mTrans[1] -= 0.3;

// 		mTrans *=1000.0;
		
// 	}
// }
void Window::SetFocusing()
{
	Eigen::Vector3d target = mEnv->GetCharacter()->GetSkeleton()->getCOM();
	Eigen::Vector3d eye;
	Eigen::Vector3d up(0, 1, 0);

	switch (mViewIndex % 4)
	{
		case 0:  // 前视角（面对人物）
			eye = target + Eigen::Vector3d(0.0, 0.5, -2.0);
			break;
		case 1:  // 后视角（从背后看）
			eye = target + Eigen::Vector3d(0.0, 0.5, 2.0);
			break;
		case 2:  // 左侧视角
			eye = target + Eigen::Vector3d(-2.0, 0.5, 0.0);
			break;
		case 3:  // 右侧视角
			eye = target + Eigen::Vector3d(2.0, 0.5, 0.0);
			break;
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye[0], eye[1], eye[2],
			  target[0], target[1], target[2],
			  up[0], up[1], up[2]);
}



Eigen::VectorXd
Window::
GetActionFromNN()
{
	return nn_module.attr("get_action")(mEnv->GetState()).cast<Eigen::VectorXd>();
}

Eigen::VectorXd
Window::
GetActivationFromNN(const Eigen::VectorXd& mt)
{
	if(!mMuscleNNLoaded)
	{
		mEnv->GetDesiredTorques();
		return Eigen::VectorXd::Zero(mEnv->GetCharacter()->GetMuscles().size());
	}
	py::object get_activation = muscle_nn_module.attr("get_activation");

	return muscle_nn_module.attr("get_activation")(mt, mEnv->GetDesiredTorques()).cast<Eigen::VectorXd>();
}

void Window::DrawEntity(const Entity* entity, const Eigen::Vector4d& color)
{
	if (!entity) return;

	const auto& bn = dynamic_cast<const BodyNode*>(entity);
	if (bn)
	{
		DrawBodyNode(bn, color);
		return;
	}

	const auto& sf = dynamic_cast<const ShapeFrame*>(entity);
	if (sf)
	{
		DrawShapeFrame(sf, color);
		return;
	}
}

void
Window::
DrawEntity(const Entity* entity)
{
	if (!entity)
		return;
	const auto& bn = dynamic_cast<const BodyNode*>(entity);
	if(bn)
	{
		DrawBodyNode(bn);
		return;
	}

	const auto& sf = dynamic_cast<const ShapeFrame*>(entity);
	if(sf)
	{
		DrawShapeFrame(sf);
		return;
	}
}


void Window::DrawBodyNode(const BodyNode* bn, const Eigen::Vector4d& color)
{
	if(!bn || !mRI) return;

	mRI->pushMatrix();
	mRI->transform(bn->getRelativeTransform());

	auto sns = bn->getShapeNodesWith<VisualAspect>();
	for(const auto& sn : sns)
		DrawShapeFrame(sn, color);  // 加颜色

	for(const auto& et : bn->getChildEntities())
		DrawEntity(et, color);

	mRI->popMatrix();
}

void
Window::
DrawBodyNode(const BodyNode* bn)
{	
	if(!bn)
		return;
	if(!mRI)
		return;

	mRI->pushMatrix();
	mRI->transform(bn->getRelativeTransform());

	auto sns = bn->getShapeNodesWith<VisualAspect>();
	for(const auto& sn : sns)
		DrawShapeFrame(sn);

	for(const auto& et : bn->getChildEntities())
		DrawEntity(et);

	mRI->popMatrix();

}

void Window::DrawSkeleton(const SkeletonPtr& skel, const Eigen::Vector4d& color)
{
	DrawBodyNode(skel->getRootBodyNode(), color);
}



void
Window::
DrawSkeleton(const SkeletonPtr& skel)
{
	DrawBodyNode(skel->getRootBodyNode());
}


void
Window::
DrawShapeFrame(const ShapeFrame* sf)
{
	if(!sf)
		return;

	if(!mRI)
		return;

	const auto& va = sf->getVisualAspect();

	if(!va || va->isHidden())
		return;

	mRI->pushMatrix();
	mRI->transform(sf->getRelativeTransform());
	DrawShape(sf->getShape().get(), Eigen::Vector4d(0.98, 0.94, 0.88, 1.0));

	mRI->popMatrix();
}

void Window::DrawShapeFrame(const ShapeFrame* sf, const Eigen::Vector4d& color)
{
	if (!sf || !mRI) return;

	const auto& va = sf->getVisualAspect();
	if (!va || va->isHidden()) return;

	mRI->pushMatrix();
	mRI->transform(sf->getRelativeTransform());
	DrawShape(sf->getShape().get(), color);  // 用自定义颜色
	mRI->popMatrix();
}


void
Window::
DrawShape(const Shape* shape,const Eigen::Vector4d& color)
{
	if(!shape)
		return;
	if(!mRI)
		return;

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	mRI->setPenColor(color);
	if(mDrawOBJ == false)
	{
		if (shape->is<SphereShape>())
		{
			const auto* sphere = static_cast<const SphereShape*>(shape);
			mRI->drawSphere(sphere->getRadius());
		}
		else if (shape->is<BoxShape>())
		{
			const auto* box = static_cast<const BoxShape*>(shape);
			mRI->drawCube(box->getSize());
		}
		else if (shape->is<CapsuleShape>())
		{
			const auto* capsule = static_cast<const CapsuleShape*>(shape);
			mRI->drawCapsule(capsule->getRadius(), capsule->getHeight());
		}	
	}
	else
	{
		if (shape->is<MeshShape>())
		{
			const auto& mesh = static_cast<const MeshShape*>(shape);
			
			glEnable(GL_COLOR_MATERIAL); // 允许使用颜色
			// glColor4f(0.98f, 0.94f, 0.88f, 1.0f); // 奶白色
			// mRI->setPenColor(Eigen::Vector4d(0.98, 0.94, 0.88, 1.0)); // 也设置 DART 内部的笔颜色
		
			mRI->drawMesh(mesh->getScale(), mesh->getMesh());
		}
		
	}
	
	glDisable(GL_COLOR_MATERIAL);
}
void Window::DrawMuscles(const std::vector<Muscle*>& muscles)
{
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);  // 
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // 

    for (auto muscle : muscles)
    {
        auto aps = muscle->GetAnchors();
        double a = muscle->activation;

        // 颜色插值：从蓝 (低) -> 橙 (中) -> 红 (高)
        Eigen::Vector4d color;
		if (a <= 0.2)
        {
			continue;
        }
        else if (a <= 0.5)
        {
            // 蓝色到橙色
            double t = a / 0.5;
            color = Eigen::Vector4d(
                (1.0 - t) * 0.0 + t * 1.0,  // R: 0 → 1
                (1.0 - t) * 0.0 + t * 0.5,  // G: 0 → 0.5
                (1.0 - t) * 1.0 + t * 0.0,  // B: 1 → 0
                1.0                         // Alpha
            );
        }
        else
        {
            // 橙色到红色
            double t = (a - 0.5) / 0.5;
            color = Eigen::Vector4d(
                1.0,                        // R: 保持 1
                (1.0 - t) * 0.5 + t * 0.0,  // G: 0.5 → 0
                0.0,                        // B: 保持 0
                1.0                         // Alpha
            );
        }

        mRI->setPenColor(color);

        // Draw anchor spheres
        for (int i = 0; i < aps.size(); i++)
        {
            Eigen::Vector3d p = aps[i]->GetPoint();
            mRI->pushMatrix();
            mRI->translate(p);
            mRI->drawSphere(0.005 * sqrt(muscle->f0 / 1000.0));
            mRI->popMatrix();
        }

        // Draw muscle fibers (cylinders)
        for (int i = 0; i < aps.size() - 1; i++)
        {
            Eigen::Vector3d p = aps[i]->GetPoint();
            Eigen::Vector3d p1 = aps[i + 1]->GetPoint();
            Eigen::Vector3d u(0, 0, 1);
            Eigen::Vector3d v = p - p1;
            Eigen::Vector3d mid = 0.5 * (p + p1);
            double len = v.norm();
            v /= len;

            Eigen::Isometry3d T;
            T.setIdentity();
            Eigen::Vector3d axis = u.cross(v);
            axis.normalize();
            double angle = acos(u.dot(v));
            Eigen::Matrix3d w_bracket = Eigen::Matrix3d::Zero();
            w_bracket(0, 1) = -axis(2);
            w_bracket(1, 0) =  axis(2);
            w_bracket(0, 2) =  axis(1);
            w_bracket(2, 0) = -axis(1);
            w_bracket(1, 2) = -axis(0);
            w_bracket(2, 1) =  axis(0);

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
                                + sin(angle) * w_bracket
                                + (1.0 - cos(angle)) * w_bracket * w_bracket;
            T.linear() = R;
            T.translation() = mid;
            mRI->pushMatrix();
            mRI->transform(T);
            mRI->drawCylinder(0.005 * sqrt(muscle->f0 / 1000.0), len);
            mRI->popMatrix();
        }
    }

    glEnable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
}
void Window::DrawMuscles1(const std::vector<Muscle*>& muscles)
{
	std::vector<std::string> muscle_name_whitelist = {
		"L_Bicep_Femoris_Longus", "R_Bicep_Femoris_Longus",
		"L_Bicep_Femoris_Short", "R_Bicep_Femoris_Short",
		"L_Bicep_Femoris_Short1", "R_Bicep_Femoris_Short1",
		"L_Rectus_Femoris", "R_Rectus_Femoris",
		"L_Rectus_Femoris1", "R_Rectus_Femoris1",
		"L_Gluteus_Maximus", "R_Gluteus_Maximus",
		"L_Gluteus_Maximus1", "R_Gluteus_Maximus1",
		"L_Gluteus_Maximus2", "R_Gluteus_Maximus2",
		"L_Gluteus_Maximus3", "R_Gluteus_Maximus3",
		"L_Gluteus_Maximus4", "R_Gluteus_Maximus4",
		"L_Gluteus_Medius", "R_Gluteus_Medius",
		"L_Gluteus_Medius1", "R_Gluteus_Medius1",
		"L_Gluteus_Medius2", "R_Gluteus_Medius2",
		"L_Gluteus_Medius3", "R_Gluteus_Medius3",
		"L_Semitendinosus", "R_Semitendinosus",
		"L_Semimembranosus", "R_Semimembranosus",
		"L_Semimembranosus1", "R_Semimembranosus1"
	};

	
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);  // 
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // 

    for (auto muscle : muscles)
    {
		std::string name = muscle->name;
		bool keep = std::find(muscle_name_whitelist.begin(), muscle_name_whitelist.end(), name) != muscle_name_whitelist.end();
		if (!keep)
			continue;

        auto aps = muscle->GetAnchors();
        double a = muscle->activation;

        // 颜色插值：从蓝 (低) -> 橙 (中) -> 红 (高)
		Eigen::Vector3d base_color(0.5, 0.5, 0.5); // 默认灰色
		double alpha = a; // 激活度为透明度

		//if (a <= 0.1)
			//continue; // 跳过显示

		// 颜色分类（手动硬编码）
		if (name.find("Bicep_Femoris") != std::string::npos)
			base_color = Eigen::Vector3d(0.2, 0.4, 1.0);  // 蓝紫色
		else if (name.find("Rectus_Femoris") != std::string::npos)
			base_color = Eigen::Vector3d(1.0, 0.7, 0.2);  // 橘黄
		else if (name.find("Gluteus_Maximus") != std::string::npos)
			base_color = Eigen::Vector3d(1.0, 0.3, 0.3);  // 红
		else if (name.find("Gluteus_Medius") != std::string::npos)
			base_color = Eigen::Vector3d(0.6, 0.4, 1.0);  // 紫
		else if (name.find("Semitendinosus") != std::string::npos ||
				name.find("Semimembranosus") != std::string::npos)
			base_color = Eigen::Vector3d(0.0, 0.6, 0.4);  // 橙

		// 构造 RGBA 向量
		Eigen::Vector4d color(base_color[0], base_color[1], base_color[2], alpha);
		base_color *= 0.75; // 全体统一加深

        mRI->setPenColor(color);

        // Draw anchor spheres
        for (int i = 0; i < aps.size(); i++)
        {
            Eigen::Vector3d p = aps[i]->GetPoint();
            mRI->pushMatrix();
            mRI->translate(p);
            mRI->drawSphere(0.005 * sqrt(muscle->f0 / 1000.0));
            mRI->popMatrix();
        }

        // Draw muscle fibers (cylinders)
        for (int i = 0; i < aps.size() - 1; i++)
        {
            Eigen::Vector3d p = aps[i]->GetPoint();
            Eigen::Vector3d p1 = aps[i + 1]->GetPoint();
            Eigen::Vector3d u(0, 0, 1);
            Eigen::Vector3d v = p - p1;
            Eigen::Vector3d mid = 0.5 * (p + p1);
            double len = v.norm();
            v /= len;

            Eigen::Isometry3d T;
            T.setIdentity();
            Eigen::Vector3d axis = u.cross(v);
            axis.normalize();
            double angle = acos(u.dot(v));
            Eigen::Matrix3d w_bracket = Eigen::Matrix3d::Zero();
            w_bracket(0, 1) = -axis(2);
            w_bracket(1, 0) =  axis(2);
            w_bracket(0, 2) =  axis(1);
            w_bracket(2, 0) = -axis(1);
            w_bracket(1, 2) = -axis(0);
            w_bracket(2, 1) =  axis(0);

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity()
                                + sin(angle) * w_bracket
                                + (1.0 - cos(angle)) * w_bracket * w_bracket;
            T.linear() = R;
            T.translation() = mid;
            mRI->pushMatrix();
            mRI->transform(T);
            mRI->drawCylinder(0.005 * sqrt(muscle->f0 / 1000.0), len);
            mRI->popMatrix();
        }
    }

    glEnable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
}

void
Window::
DrawShadow(const Eigen::Vector3d& scale, const aiScene* mesh,double y) 
{
	glDisable(GL_LIGHTING);
	glPushMatrix();
	glScalef(scale[0],scale[1],scale[2]);
	GLfloat matrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	Eigen::Matrix3d A;
	Eigen::Vector3d b;
	A<<matrix[0],matrix[4],matrix[8],
	matrix[1],matrix[5],matrix[9],
	matrix[2],matrix[6],matrix[10];
	b<<matrix[12],matrix[13],matrix[14];

	Eigen::Affine3d M;
	M.linear() = A;
	M.translation() = b;
	M = (mViewMatrix.inverse()) * M;

	glPushMatrix();
	glLoadIdentity();
	glMultMatrixd(mViewMatrix.data());
	DrawAiMesh(mesh,mesh->mRootNode,M,y);
	glPopMatrix();
	glPopMatrix();
	glEnable(GL_LIGHTING);
}
void
Window::
DrawAiMesh(const struct aiScene *sc, const struct aiNode* nd,const Eigen::Affine3d& M,double y)
{
	unsigned int i;
    unsigned int n = 0, t;
    Eigen::Vector3d v;
    Eigen::Vector3d dir(0.4,0,-0.4);
	glDisable(GL_TEXTURE_2D);       // 不用纹理
	glDisable(GL_LIGHTING);         // 不用光照
	glDisable(GL_COLOR_MATERIAL);   // 关闭颜色-材质混合
	glColor4f(0.94, 0.92, 0.85, 1.0); // 奶白色 + 不透明
	

    
    // update transform

    // draw all meshes assigned to this node
    for (; n < nd->mNumMeshes; ++n) {
        const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

        for (t = 0; t < mesh->mNumFaces; ++t) {
            const struct aiFace* face = &mesh->mFaces[t];
            GLenum face_mode;

            switch(face->mNumIndices) {
                case 1: face_mode = GL_POINTS; break;
                case 2: face_mode = GL_LINES; break;
                case 3: face_mode = GL_TRIANGLES; break;
                default: face_mode = GL_POLYGON; break;
            }
            glBegin(face_mode);
        	for (i = 0; i < face->mNumIndices; i++)
        	{
        		int index = face->mIndices[i];

        		v[0] = (&mesh->mVertices[index].x)[0];
        		v[1] = (&mesh->mVertices[index].x)[1];
        		v[2] = (&mesh->mVertices[index].x)[2];
        		v = M*v;
        		double h = v[1]-y;
        		
        		v += h*dir;
        		
        		v[1] = y+0.001;
        		glVertex3f(v[0],v[1],v[2]);
        	}
            glEnd();
        }

    }

    // draw all children
    for (n = 0; n < nd->mNumChildren; ++n) {
        DrawAiMesh(sc, nd->mChildren[n],M,y);
    }

}
void
Window::
DrawGround(double y)
{
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glDisable(GL_LIGHTING);
	double width = 0.005;
	int count = 0;
	glBegin(GL_QUADS);
	for(double x = -100.0;x<100.01;x+=1.0)
	{
		for(double z = -100.0;z<100.01;z+=1.0)
		{
			if(count%2==0)
				glColor3f(216.0/255.0,211.0/255.0,204.0/255.0);			
			else
				glColor3f(216.0/255.0-0.1,211.0/255.0-0.1,204.0/255.0-0.1);
			count++;
			glVertex3f(x,y,z);
			glVertex3f(x+1.0,y,z);
			glVertex3f(x+1.0,y,z+1.0);
			glVertex3f(x,y,z+1.0);
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);
}

void Window::SaveScreenshot() {
	static int screenshot_counter = 0;
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	int width = viewport[2];
	int height = viewport[3];

	unsigned char* pixels = new unsigned char[3 * width * height];
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	// 垂直翻转图像
	for (int j = 0; j < height / 2; ++j) {
		for (int i = 0; i < width * 3; ++i) {
			std::swap(pixels[j * width * 3 + i], pixels[(height - j - 1) * width * 3 + i]);
		}
	}

	char filename[128];
	sprintf(filename, "screenshot_%03d.ppm", screenshot_counter++);
	FILE* fp = fopen(filename, "wb");
	fprintf(fp, "P6\n%d %d\n255\n", width, height);
	fwrite(pixels, 1, width * height * 3, fp);
	fclose(fp);
	delete[] pixels;

	std::cout << "Saved screenshot: " << filename << std::endl;
}
