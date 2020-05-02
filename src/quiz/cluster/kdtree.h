/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node==NULL){
			*node = (new Node(point, id));
		}
		else
		{
			char dim = depth % 2;
			if( point[dim] < (*node)->point[dim] )
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			// is the point inside the box?
			uint dim = depth%2;
			std::cout << "Current  dimension is: " << dim << std::endl;
			std::cout << "left or bottom of box is: " << node->point[dim]-distanceTol << std::endl;
			std::cout << "right or top of box is: " << node->point[dim]+distanceTol << std::endl;
			std::cout << "target dim is: " << target[dim] << std::endl;
			if( (node->point[0]-distanceTol <= target[0])&&(node->point[0]+distanceTol >= target[0]) && (node->point[1]-distanceTol <= target[1])&&(node->point[1]+distanceTol >= target[1]))
			{
				// calculate the distance
				float distance = sqrt(pow(fabs(node->point[0]-target[0]),2) + pow(fabs(node->point[1]-target[1]), 2));
				std::cout << "The real calculated distance was: " << distance << std::endl;
				if (distance <= distanceTol)
				{
					// add the point to the list
					ids.push_back(node->id);
					std::cout << "Found an inlier with id: " << node->id << std::endl;
				}
			}

			// checked left box boundary is smaller than current node, check left
			if( (target[dim] - distanceTol) < node->point[dim] )
			{
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			// checked right box boundary is larger than current node, check right
			if ( (target[dim] + distanceTol) > node->point[dim] )
			{
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};




