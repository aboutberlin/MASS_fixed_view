#include "Character.h"
#include "BVH.h"
#include "DARTHelper.h"
#include "Muscle.h"
#include <tinyxml.h>
#include <stdio.h>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
using namespace dart;
using namespace dart::dynamics;
using namespace MASS;
Character::
Character()
	:mSkeleton(nullptr),mBVH(nullptr),mTc(Eigen::Isometry3d::Identity())
{

}

void
Character::
LoadExofromUrdf(const std::string& path,bool create_obj)
{
	if(path.substr(path.size()-5,5) == ".urdf") {
		dart::utils::DartLoader loader; // load .urdf file
		mExo = loader.parseSkeleton(path); //the name shall be loaded inside
		mExo->setName("Hip_Exoskeleton");
		// std::cout << "skel_name:  " << mExo->getNumDofs() << std::endl;    
		}

	else {
		throw std::runtime_error("do not know how to load skeleton " + path);
	}

   
    mExodof = mExo->getNumDofs();
	mExoJoints = mExo->getNumJoints();
	mExobodynodes = mExo->getNumBodyNodes();
	mhumanbodynodes=0;
	std::cout << "[DEBUG] mExo body nodes: " << mExo->getNumBodyNodes() << std::endl;

}



void
Character::
MergeHumanandExo()
{
	bool mergesuccess = false;
	mergesuccess = mExo->getRootBodyNode()->moveTo<dart::dynamics::WeldJoint>(mSkeleton, mSkeleton->getRootBodyNode());
	Joint* parentJoint = mSkeleton->getBodyNode("exo_waist")->getParentJoint();
	Eigen::Isometry3d T = parentJoint->getTransformFromParentBodyNode();
	T.linear() = T.linear()* R_y(-90);
	T.linear() = T.linear()* R_x(-3);
	// T.translation()(0) = T.translation()(0)+0.013;
	T.translation()(1) = T.translation()(1)+0.1;
	T.translation()(2) = T.translation()(2)+0.0;

	parentJoint->setTransformFromParentBodyNode(T);  
	if (mergesuccess ==false)
	{
		throw std::runtime_error("Something Wrong in Merge");
	}
}

// ====== 旧函数删掉，换成下面这一版 ======
void Character::AlignExoToHuman(const Eigen::Vector3d& trans_off,
	const Eigen::Vector3d& rot_deg)
{
if (!mSkeleton || !mExo) return;

// 两棵 Skeleton 的根关节都应是 FreeJoint
auto humanRoot = dynamic_cast<dart::dynamics::FreeJoint*>(mSkeleton->getJoint(0));
auto exoRoot   = dynamic_cast<dart::dynamics::FreeJoint*>(mExo->getJoint(0));
if (!humanRoot || !exoRoot) return;

/* ---------- 复制人体的 6‑DoF 位姿 ---------- */
Eigen::Vector6d pose_human = humanRoot->getPositions();
Eigen::Isometry3d T_human  = dart::dynamics::FreeJoint::convertToTransform(pose_human);

/* ---------- 构造用户偏移变换 ---------- */
Eigen::Isometry3d T_off = Eigen::Isometry3d::Identity();
T_off.translation() = trans_off;

// 旋转偏移（输入为度，转弧度后按 Z‑Y‑X 乘）
double rx = rot_deg[0] * M_PI / 180.0;
double ry = rot_deg[1] * M_PI / 180.0;
double rz = rot_deg[2] * M_PI / 180.0;
T_off.linear() = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix() *
Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();

/* ---------- 应用偏移并写回外骨骼 ---------- */
Eigen::Isometry3d T_exo = T_human * T_off;
Eigen::Vector6d    pose_exo = dart::dynamics::FreeJoint::convertToPositions(T_exo);

exoRoot->setPositions(pose_exo);
mExo->computeForwardKinematics(true, false, false);
}





void
Character::
LoadSkeleton(const std::string& path,bool create_obj)
{
	mSkeleton = BuildFromFile(path,create_obj);
	std::map<std::string,std::string> bvh_map;
	TiXmlDocument doc;
	doc.LoadFile(path);
	TiXmlElement *skel_elem = doc.FirstChildElement("Skeleton");

	for(TiXmlElement* node = skel_elem->FirstChildElement("Node");node != nullptr;node = node->NextSiblingElement("Node"))
	{
		if(node->Attribute("endeffector")!=nullptr)
		{
			std::string ee =node->Attribute("endeffector");
			if(ee == "True")
			{
				mEndEffectors.push_back(mSkeleton->getBodyNode(std::string(node->Attribute("name"))));
			}
		}
		TiXmlElement* joint_elem = node->FirstChildElement("Joint");
		if(joint_elem->Attribute("bvh")!=nullptr)
		{
			bvh_map.insert(std::make_pair(node->Attribute("name"),joint_elem->Attribute("bvh")));
		}
	}
	
	mBVH = new BVH(mSkeleton,bvh_map);
}
void
Character::
LoadMuscles(const std::string& path)
{
	TiXmlDocument doc;
	if(!doc.LoadFile(path)){
		std::cout << "Can't open file : " << path << std::endl;
		return;
	}

	TiXmlElement *muscledoc = doc.FirstChildElement("Muscle");
	for(TiXmlElement* unit = muscledoc->FirstChildElement("Unit");unit!=nullptr;unit = unit->NextSiblingElement("Unit"))
	{
		std::string name = unit->Attribute("name");
		double f0 = std::stod(unit->Attribute("f0"));
		double lm = std::stod(unit->Attribute("lm"));
		double lt = std::stod(unit->Attribute("lt"));
		double pa = std::stod(unit->Attribute("pen_angle"));
		double lmax = std::stod(unit->Attribute("lmax"));
		mMuscles.push_back(new Muscle(name,f0,lm,lt,pa,lmax));
		int num_waypoints = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
			num_waypoints++;
		int i = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
		{
			std::string body = waypoint->Attribute("body");
			Eigen::Vector3d glob_pos = string_to_vector3d(waypoint->Attribute("p"));
			if(i==0||i==num_waypoints-1)
			// if(true)
				mMuscles.back()->AddAnchor(mSkeleton->getBodyNode(body),glob_pos);
			else
				mMuscles.back()->AddAnchor(mSkeleton,mSkeleton->getBodyNode(body),glob_pos,2);

			i++;
		}
	}
	

}
void
Character::
LoadBVH(const std::string& path,bool cyclic)
{
	if(mBVH ==nullptr){
		std::cout<<"Initialize BVH class first"<<std::endl;
		return;
	}
	mBVH->Parse(path,cyclic);
}
void
Character::
Reset()
{
	mTc = mBVH->GetT0();
	mTc.translation()[1] = 0.0;
}
void
Character::
SetPDParameters(double kp, double kv)
{
	int dof = mSkeleton->getNumDofs();
	mKp = Eigen::VectorXd::Constant(dof,kp);	
	mKv = Eigen::VectorXd::Constant(dof,kv);	
}
Eigen::VectorXd
Character::
GetSPDForces(const Eigen::VectorXd& p_desired)
{
	Eigen::VectorXd q = mSkeleton->getPositions();
	Eigen::VectorXd dq = mSkeleton->getVelocities();
	double dt = mSkeleton->getTimeStep();
	// Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix();
	Eigen::MatrixXd M_inv = (mSkeleton->getMassMatrix() + Eigen::MatrixXd(dt*mKv.asDiagonal())).inverse();

	Eigen::VectorXd qdqdt = q + dq*dt;

	Eigen::VectorXd p_diff = -mKp.cwiseProduct(mSkeleton->getPositionDifferences(qdqdt,p_desired));
	Eigen::VectorXd v_diff = -mKv.cwiseProduct(dq);
	Eigen::VectorXd ddq = M_inv*(-mSkeleton->getCoriolisAndGravityForces()+p_diff+v_diff+mSkeleton->getConstraintForces());

	Eigen::VectorXd tau = p_diff + v_diff - dt*mKv.cwiseProduct(ddq);

	tau.head<6>().setZero();

	return tau;
}
Eigen::VectorXd
Character::
GetTargetPositions(double t,double dt)
{
	// std::cout<<"GetTargetPositions"<<std::endl;
	Eigen::VectorXd p = mBVH->GetMotion(t);	
	Eigen::Isometry3d T_current = dart::dynamics::FreeJoint::convertToTransform(p.head<6>());
	T_current = mBVH->GetT0().inverse()*T_current;
	Eigen::Isometry3d T_head = mTc*T_current;
	Eigen::Vector6d p_head = dart::dynamics::FreeJoint::convertToPositions(T_head);
	p.head<6>() = p_head;
	
	
	if(mBVH->IsCyclic())
	{
		double t_mod = std::fmod(t, mBVH->GetMaxTime());
		t_mod = t_mod/mBVH->GetMaxTime();

		double r = 0.95;
		if(t_mod>r)
		{
			double ratio = 1.0/(r-1.0)*t_mod - 1.0/(r-1.0);
			Eigen::Isometry3d T01 = mBVH->GetT1()*(mBVH->GetT0().inverse());
			double delta = T01.translation()[1];
			delta *= ratio;
			p[5] += delta;
		}



		double tdt_mod = std::fmod(t+dt, mBVH->GetMaxTime());
		if(tdt_mod-dt<0.0){
			Eigen::Isometry3d T01 = mBVH->GetT1()*(mBVH->GetT0().inverse());
			Eigen::Vector3d p01 = dart::math::logMap(T01.linear());
			p01[0] =0.0;
			p01[2] =0.0;
			T01.linear() = dart::math::expMapRot(p01);

			mTc = T01*mTc;
			mTc.translation()[1] = 0.0;
		}
	}
	

	return p;
}

std::pair<Eigen::VectorXd,Eigen::VectorXd>
Character::
GetTargetPosAndVel(double t,double dt)
{
	Eigen::VectorXd p = this->GetTargetPositions(t,dt);
	Eigen::Isometry3d Tc = mTc;
	Eigen::VectorXd p1 = this->GetTargetPositions(t+dt,dt);
	mTc = Tc;

	return std::make_pair(p,(p1-p)/dt);
}