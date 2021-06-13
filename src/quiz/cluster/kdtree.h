/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cstdlib>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
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

	void insertHelper3D(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node==NULL) {
				*node = new Node(point,id);
				std::cout << "new node added. ID: " << id << " Point: " << point[0] << ", " << point[1] << ", " << point[2] << '\n';
		}
		else {
			uint CurrentDepth = depth % 3;

			if (point[CurrentDepth] < ((*node)->point[CurrentDepth])) {
				std::cout << "going left: " << ((*node)->id) << " => ";
				if (((*node)->left)==NULL){
					std::cout << ((*node)->left) << '\n';
				}
				else {
					std::cout << (((*node)->left)->id) << '\n';
				}
				insertHelper3D(&((*node)->left), depth+1, point, id);
			}
			else {
				std::cout << "going right: " << ((*node)->id) << " => ";
				if (((*node)->right)==NULL){
					std::cout << ((*node)->right) << '\n';
				}
				else {
					std::cout << (((*node)->right)->id) << '\n';
				}
				insertHelper3D(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert3D(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root
		std::cout << "insert helper called.	Root: " << root << '\n';
		insertHelper3D(&root, 0, point , id);
	}

	void search_helper3D(std::vector<float> target, Node *root, int depth, float distanceTol, std::vector<int> &ids)
	{
		// std::cout << "target x:" << target[0] << '\n';
		// std::cout << "node value x:" << root->point[0] << '\n';
		if (root != NULL)
		{ //check if node is inside box
			if ((root->point[0] >= (target[0] - distanceTol) && root->point[0] <= (target[0] + distanceTol)) &&
								(root->point[1] >= (target[1] - distanceTol) && root->point[1] <= (target[1] + distanceTol)))
			{
				std::cout << "Node: " << root->id << " inside box" << '\n';
				float point_dist = sqrt((root->point[0] - target[0]) * (root->point[0] - target[0]) +
																			(root->point[1] - target[1]) * (root->point[1] - target[1]));
				if (point_dist <= distanceTol)
				{
					std::cout << "Node: " << root->id << " inside radius" << '\n';
					ids.push_back(root->id);
					std::cout << "ids appended: ";
					for (std::vector<int>::const_iterator i = ids.begin(); i != ids.end(); ++i)
					{
						std::cout << *i << ", ";
					}

				}
			}
			std::cout << "picking a path" << '\n';
			if ((target[depth % 2] - distanceTol) < root->point[depth % 2])
			{
				std::cout << "going left: " << root->id << " => ";
				if ((root->left)==NULL){
					std::cout << root->left << '\n';
				}
				else {
					std::cout << ((root->left)->id) << '\n';
				}
				search_helper3D(target, root->left, depth + 1, distanceTol, ids);
			}

			if ((target[depth % 2] + distanceTol) > root->point[depth % 2])
			{
				std::cout << "going right: " << root->id << " => ";
				if ((root->right)==NULL){
					std::cout << root->right << '\n';
				}
				else {
					std::cout << ((root->right)->id) << '\n';
				}
				search_helper3D(target, root->right, depth + 1, distanceTol, ids);
			}

		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search3D(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper3D(target, root, 0, distanceTol, ids);
		//even tho root is a global, we want it to be passed through the helper function for recursion purposes
		return ids;
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node==NULL) {
			*node = new Node(point,id);
			std::cout << "new node added. ID: " << id << '\n';
		}
		else{
			uint CurrentDepth = depth % 2;

			if (point[CurrentDepth] < ((*node)->point[CurrentDepth])) {
				std::cout << "going left: " << ((*node)->id) << " => ";
				if (((*node)->left)==NULL){
					std::cout << ((*node)->left) << '\n';
				}
				else {
					std::cout << (((*node)->left)->id) << '\n';
				}
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else {
				std::cout << "going right: " << ((*node)->id) << " => ";
				if (((*node)->right)==NULL){
					std::cout << ((*node)->right) << '\n';
				}
				else {
					std::cout << (((*node)->right)->id) << '\n';
				}
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// the function should create a new node and place correctly with in the root
		std::cout << "insert helper called.	Root: " << root << '\n';
		insertHelper(&root, 0, point , id);
	}

	void search_helper(std::vector<float> target, Node *root, int depth, float distanceTol, std::vector<int> &ids)
	{
		// std::cout << "target x:" << target[0] << '\n';
		// std::cout << "node value x:" << root->point[0] << '\n';
		if (root != NULL)
		{ //check if node is inside box
			if ((root->point[0] >= (target[0] - distanceTol) && root->point[0] <= (target[0] + distanceTol)) &&
                (root->point[1] >= (target[1] - distanceTol) && root->point[1] <= (target[1] + distanceTol)))
			{
				std::cout << "Node: " << root->id << " inside box" << '\n';
				float point_dist = sqrt((root->point[0] - target[0]) * (root->point[0] - target[0]) +
                                      (root->point[1] - target[1]) * (root->point[1] - target[1]));
				if (point_dist <= distanceTol)
				{
					std::cout << "Node: " << root->id << " inside radius" << '\n';
					ids.push_back(root->id);
					std::cout << "ids appended: ";
					for (std::vector<int>::const_iterator i = ids.begin(); i != ids.end(); ++i)
					{
						std::cout << *i << ", ";
					}

				}
			}
			std::cout << "picking a path" << '\n';
			if ((target[depth % 2] - distanceTol) < root->point[depth % 2])
			{
				std::cout << "going left: " << root->id << " => ";
				if ((root->left)==NULL){
					std::cout << root->left << '\n';
				}
				else {
					std::cout << ((root->left)->id) << '\n';
				}
				search_helper(target, root->left, depth + 1, distanceTol, ids);
			}

			if ((target[depth % 2] + distanceTol) > root->point[depth % 2])
			{
				std::cout << "going right: " << root->id << " => ";
				if ((root->right)==NULL){
					std::cout << root->right << '\n';
				}
				else {
					std::cout << ((root->right)->id) << '\n';
				}
				search_helper(target, root->right, depth + 1, distanceTol, ids);
			}

		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(target, root, 0, distanceTol, ids);
		//even tho root is a global, we want it to be passed through the helper function for recursion purposes
		return ids;
	}

};
