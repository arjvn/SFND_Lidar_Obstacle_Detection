#ifndef KDTREE_H
#define KDTREE_H

#include <cstdlib>

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root; // tree root is constant

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
  }

	void insertHelper3D(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if (*node==NULL) {
			*node = new Node(point,id);
		}
    else {
      int CurrentDepth = depth % 3;
      if (CurrentDepth == 0){
        if (point.x < (*node)->point.x) {
          insertHelper3D(&((*node)->left), depth+1, point, id);
        }
        else {
          insertHelper3D(&((*node)->right), depth+1, point, id);
        }
      }
      else if (CurrentDepth == 1) {
        if (point.y < (*node)->point.y) {
          insertHelper3D(&((*node)->left), depth+1, point, id);
        }
        else {
          insertHelper3D(&((*node)->right), depth+1, point, id);
        }
      }
  		else {
        if (point.z < (*node)->point.z) {
          insertHelper3D(&((*node)->left), depth+1, point, id);
        }
        else {
          insertHelper3D(&((*node)->right), depth+1, point, id);
        }
  		}
    }
	}

	void insert3D(pcl::PointXYZI point, int id)
	{
		// the function should create a new node and place correctly with in the root
		insertHelper3D(&root, 0, point , id);
	}

	void search_helper3D(pcl::PointXYZI target, Node *root, int depth, float distanceTol, std::vector<int> &ids)
	{
		// std::cout << "target x:" << target[0] << '\n';
		// std::cout << "node value x:" << root->point[0] << '\n';
		if (root != NULL)
    {
      //check if node is inside box
			if ((root->point.x >= (target.x - distanceTol) && root->point.x <= (target.x + distanceTol)) &&
					(root->point.y >= (target.y - distanceTol) && root->point.y <= (target.y + distanceTol)) &&
          (root->point.z >= (target.z - distanceTol) && root->point.z <= (target.z + distanceTol)))
			{
				float point_dist = sqrt((root->point.x - target.x) * (root->point.x - target.x) +
																(root->point.y - target.y) * (root->point.y - target.y) +
                                (root->point.z - target.z) * (root->point.z - target.z));
				if (point_dist <= distanceTol)
				{
					ids.push_back(root->id);
				}
			}
      // Traversing the tree
      if (depth % 3 == 0) {
        if ((target.x - distanceTol) < root->point.x) {
          search_helper3D(target, root->left, depth + 1, distanceTol, ids);
        }
        if ((target.x + distanceTol) > root->point.x) {
          search_helper3D(target, root->right, depth + 1, distanceTol, ids);
        }
      }
      else if (depth % 3 == 1){
        if ((target.y - distanceTol) < root->point.y) {
          search_helper3D(target, root->left, depth + 1, distanceTol, ids);
        }
        if ((target.y + distanceTol) > root->point.y) {
          search_helper3D(target, root->right, depth + 1, distanceTol, ids);
        }
      }
      else {
        if ((target.z - distanceTol) < root->point.z) {
          search_helper3D(target, root->left, depth + 1, distanceTol, ids);
        }
        if ((target.z + distanceTol) > root->point.z) {
          search_helper3D(target, root->right, depth + 1, distanceTol, ids);
        }
      }
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search3D(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper3D(target, root, 0, distanceTol, ids);
		//even tho root is a global, we want it to be passed through the helper function for recursion purposes
		return ids;
	}
};

#endif
