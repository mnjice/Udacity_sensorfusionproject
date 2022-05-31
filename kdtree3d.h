/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#ifndef PLAYBACK_KDTREE3D_H
#define PLAYBACK_KDTREE3D_H

#include <pcl/impl/point_types.hpp>
#include <vector>

struct Node{
	pcl::PointXYZI point;
	int id;
	Node *left;
	Node *right;
	Node(pcl::PointXYZI arr,int setId):point(arr),id(setId),left(NULL),right(NULL){}

};

struct kdTree{
	Node *root;
	kdTree():root(NULL){}

	void insertHelper(Node **node, int depth,pcl::PointXYZI point, int id){
		if(*node == NULL){
			*node = new Node{point,id};
		}else{
			int cd = depth%3;
			if(cd==0){
				if(point.x<(*node)->point.x){
					insertHelper(&((*node)->left),depth+1,point,id);
				}else{
					insertHelper(&((*node)->right),depth+1,point,id);
				}
			}else if(cd == 1){
				if(point.y<(*node)->point.y){
					insertHelper(&((*node)->left),depth+1,point,id);
				}else{
					insertHelper(&((*node)->right),depth+1,point,id);
				}
			}else{
				if(point.z<(*node)->point.z){
					insertHelper(&((*node)->left),depth+1,point,id);
				}else{
					insertHelper(&((*node)->right),depth+1,point,id);
				}
			}
		}
	}

	void insert(pcl::PointXYZI point, int id){
		insertHelper(&root,0,point,id);
	}

	void searchHelper(pcl::PointXYZI target, Node *node, int depth, float distanceTol,std::vector<int> &ids){
		if(node!=NULL){
			float delta_x = node->point.x - target.x;
			float delta_y = node->point.y - target.y;
			float delta_z = node->point.z - target.z;

			if((delta_x >= -distanceTol && delta_x<=distanceTol)&& (delta_y >= -distanceTol && delta_y<=distanceTol)&&(delta_z >= -distanceTol && delta_z<=distanceTol)) {
				float distance = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
				if(distance <= distanceTol){
					ids.push_back(node->id);
				}
			}
			if(depth%3==0){
				if(delta_x>distanceTol){
					searchHelper(target,node->left,depth+1,distanceTol,ids);
				}
				if(delta_x<distanceTol){
					searchHelper(target,node->right,depth+1,distanceTol,ids);
				}
			}else if(depth%3==1){
				if(delta_y>distanceTol){
					searchHelper(target,node->left,depth+1,distanceTol,ids);
				}
				if(delta_y<distanceTol){
					searchHelper(target,node->right,depth+1,distanceTol,ids);
				}
			}else{
				if(delta_z>distanceTol){
					searchHelper(target,node->left,depth+1,distanceTol,ids);
				}
				if(delta_z<distanceTol){
					searchHelper(target,node->right,depth+1,distanceTol,ids);
				}
			}
		}

	}
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
};
#endif
