
#include "PRLStackUtilTPG.h"

class Shortcutting
{
public:
	PRLStackUtilTPG ok;
	//14 dimensions + 1 cost
	std::vector<Eigen::VectorXd> compare_paths(std::vector<Eigen::VectorXd> &left_configs, std::vector<Eigen::VectorXd> &right_configs, size_t l, size_t r, double& cost)
	{
		// std::cout<<"PRESS [ENTER] to continue: ";
		// std::cin.get();
		// std::cout<<"IN COMPARE PATHS: "<<left_configs.size()<<" "<<right_configs.size()<<" L:"<<l<<" R:"<<r<<std::endl;
		int dim = 7;
		if(left_configs.size()==l+1)
		{
			std::vector<Eigen::VectorXd> final_path;
			size_t i;
			for(i=r; i!=right_configs.size()-1;i++)
			{
				Eigen::VectorXd target_config(dim+dim);
				target_config << left_configs.at(l) , right_configs.at(i);
				cost = cost + (right_configs.at(i)-right_configs.at(i+1)).norm();
				final_path.push_back(target_config);
			}
			Eigen::VectorXd target_config(dim+dim);
			target_config << left_configs.at(l) , right_configs.at(i);
			final_path.push_back(target_config);
			std::reverse(final_path.begin(),final_path.end());
			// std::cout<<"ADDING CONFIG"<<std::endl;
			return final_path;
		}
		if(right_configs.size()==r+1)
		{
			std::vector<Eigen::VectorXd> final_path;
			size_t i;
			for(i=l; i!=left_configs.size()-1;i++)
			{
				Eigen::VectorXd target_config(dim+dim);
				target_config << left_configs.at(i) , right_configs.at(r);
				cost = cost + (left_configs.at(i)-left_configs.at(i+1)).norm();
				final_path.push_back(target_config);
			}
			Eigen::VectorXd target_config(dim+dim);
			target_config << left_configs.at(i) , right_configs.at(r);
			final_path.push_back(target_config);
			std::reverse(final_path.begin(),final_path.end());
			// std::cout<<"ADDING CONFIG"<<std::endl;
			return final_path;
		}

		Eigen::VectorXd source_config(dim+dim);
		Eigen::VectorXd target_config(dim+dim);
		source_config << left_configs.at(l) , right_configs.at(r);
		target_config << left_configs.at(l+1) , right_configs.at(r+1);
		// std::cout<<"getting collision status: ";

		bool col = ok.getCollisionStatus(source_config,target_config);
		// std::cout<<"col: "<<col<<std::endl;
		// std::cout<<"PRESS [ENTER] to continue: ";
		// std::cin.get();
		if (!col)
		{
			cost = cost + std::max((left_configs.at(l+1)-left_configs.at(l)).norm(),
				(right_configs.at(r+1)-right_configs.at(r)).norm());
			std::vector<Eigen::VectorXd> final_path;
			final_path = compare_paths(left_configs,right_configs,l+1,r+1,cost);
			// std::cout<<"PRESS [ENTER] to continue: ";
			// std::cin.get();
			final_path.push_back(source_config);
			// std::cout<<"ADDING CONFIG"<<std::endl;
			// std::cout<<"Size of final path: "<<final_path.size();
			return final_path;
		}
		else
		{
			double cost1=0.0;
			double cost2=0.0;

			std::vector<Eigen::VectorXd> final_path1;
			// std::cout<<"CALLING finalpath1"<<std::endl;
			final_path1 = compare_paths(left_configs,right_configs,l+1,r,cost1);
			std::vector<Eigen::VectorXd> final_path2;
			// std::cout<<"CALLING finalpath2"<<std::endl;
			final_path2 = compare_paths(left_configs,right_configs,l,r+1,cost2);
			if(cost1 < cost2) // moving in left
			{
				final_path1.push_back(source_config);
				// std::cout<<"ADDING CONFIG"<<std::endl;
				// cost = cost1 + (left_configs.at(l+1)-left_configs.at(r)).norm()+ PAUSE;
				cost = cost1 + (left_configs.at(l+1)-left_configs.at(r)).norm();
				// std::cout<<"Size of final path1: "<<final_path1.size();
				return final_path1;
			}
			else //moving in right
			{
				final_path2.push_back(source_config);
				// std::cout<<"ADDING CONFIG"<<std::endl;
				// cost = cost2 + (right_configs.at(r+1)-right_configs.at(r)).norm() +PAUSE;
				cost = cost2 + (right_configs.at(r+1)-right_configs.at(r)).norm();
				// std::cout<<"Size of final path2: "<<final_path2.size();
				return final_path2;
			}
		}
	}

	std::vector<Eigen::VectorXd> shortcut_path(std::vector<Eigen::VectorXd> configs)
	{
		std::cout<<"INPUT_PATH: "<<std::endl;
		for( size_t pl=0;pl<configs.size();pl++ )
		{
			for(int num =0; num<14;num++)
				std::cout<<configs.at(pl)(num)<<" ";
			std::cout<<std::endl;
		}
		std::cout<<"IN shortcut_path"<<std::endl;
		size_t dim=7;
		std::vector<Eigen::VectorXd> left_configs;
		std::vector<Eigen::VectorXd> right_configs;
		std::cout<<"Size:"<<configs.size()<<std::endl;

		left_configs.push_back(configs.at(0).segment(0,dim));
		right_configs.push_back(configs.at(0).segment(dim,dim));

		for(size_t i=0;i<configs.size()-1;i++)
		{
			// std::cout<<"   Right size: "<< right_configs.size()<<std::endl;
			// std::cout<<"   Left Size: "<<left_configs.size()<<std::endl;
			Eigen::VectorXd source_config(dim+dim);
			Eigen::VectorXd target_config(dim+dim);

			source_config = configs.at(i);
			target_config = configs.at(i+1);

			Eigen::VectorXd left_source(dim);
			Eigen::VectorXd left_target(dim);
					
			Eigen::VectorXd right_source(dim);
			Eigen::VectorXd right_target(dim);

			left_source << source_config.segment(0,dim);
			right_source << source_config.segment(dim,dim);

			left_target << target_config.segment(0,dim);
			right_target << target_config.segment(dim,dim);


			if(left_source.isApprox(left_target))
			{
				std::cout<<"Right Arm movement! "<<std::endl;
				right_configs.push_back(right_target);
				// std::cout<<"Right size: "<< right_configs.size()<<std::endl;
			}
			else
			{
				std::cout<<"Left Arm movement! "<<std::endl;
				left_configs.push_back(left_target);
				// std::cout<<"Left Size: "<<left_configs.size()<<std::endl;
			}
		}
		std::cout<<"Size of arrays: "<<left_configs.size()<<" "<<right_configs.size()<<std::endl;
		std::vector<Eigen::VectorXd> final_path1;
		double cost=0.0;
		final_path1=compare_paths(left_configs,right_configs,0,0,cost);

		std::reverse(final_path1.begin(),final_path1.end());

		std::cout<<"FINAL_PATH: "<<std::endl;
		for( size_t pl=0;pl<final_path1.size();pl++ )
		{
			for(int num =0; num<14;num++)
				std::cout<<final_path1.at(pl)(num)<<" ";
			std::cout<<std::endl;
		}

		std::cout<<"COST: "<<cost<<std::endl;
		std::cout<<"Number of nodes: "<<final_path1.size()<<std::endl;
		ok.executeTogether(final_path1);
		return final_path1;

	}
};