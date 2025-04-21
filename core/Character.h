#ifndef __MASS_CHARACTER_H__
#define __MASS_CHARACTER_H__
#include "dart/dart.hpp"

namespace MASS
{
class BVH;
class Muscle;
class Character
{
public:
	Character();

	void LoadSkeleton(const std::string& path,bool create_obj = false);
	void LoadMuscles(const std::string& path);
	void LoadBVH(const std::string& path,bool cyclic=true);

	void Reset();	
	void SetPDParameters(double kp, double kv);
	void AddEndEffector(const std::string& body_name){mEndEffectors.push_back(mSkeleton->getBodyNode(body_name));}
	Eigen::VectorXd GetSPDForces(const Eigen::VectorXd& p_desired);

	Eigen::VectorXd GetTargetPositions(double t,double dt);
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetTargetPosAndVel(double t,double dt);
	
	
	const dart::dynamics::SkeletonPtr& GetSkeleton(){return mSkeleton;}
	const std::vector<Muscle*>& GetMuscles() {return mMuscles;}
	const std::vector<dart::dynamics::BodyNode*>& GetEndEffectors(){return mEndEffectors;}
	BVH* GetBVH(){return mBVH;}
	void MergeHumanandExo();
	void AlignExoToHuman(const Eigen::Vector3d& trans, const Eigen::Vector3d& rot_deg);

	void LoadExofromUrdf(const std::string& path,bool create_obj=false);

public:
	dart::dynamics::SkeletonPtr mSkeleton;
	dart::dynamics::SkeletonPtr GetExoSkeleton() const { return mExo; }
	BVH* mBVH;
	Eigen::Isometry3d mTc;

	std::vector<Muscle*> mMuscles;
	std::vector<dart::dynamics::BodyNode*> mEndEffectors;

	Eigen::VectorXd mKp, mKv;

	// Character.h 中，加入以下成员变量声明
	dart::dynamics::SkeletonPtr mExo;

	int mExodof;
	int mExoJoints;
	int mExobodynodes;
	int mhumanbodynodes;


};
};

#endif
