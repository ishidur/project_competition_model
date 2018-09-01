#include "VirtualMap.h"
#include "Positioning/MapInfomation/Line.h"
#include "Positioning/MapInfomation/Circle.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace Utilities;
using namespace Positioning::MapInfomation;

VirtualMap::VirtualMap():node_index(0),edge_index(0)
{
	ReadMap();
}

Node* VirtualMap::GetNode(int index)
{
	//指定サイズをチェックする
	if( index < node_index ){
		//正常
	}
	else{
		//assertにする
	}
	return NodeList[index];
}

Edge* VirtualMap::GetEdge(int index)
{
	//指定サイズをチェックする
	if( index < edge_index ){
		//正常
	}
	else{
		//assertにする
	}

	return EdgeList[index];
}

void VirtualMap::ReadMap()
{
	char param_name[255];
	float param[5];

	FILE* map_file = fopen("/ev3rt/res/navigation/map_params.txt","r");

	if( map_file == NULL ){
//		assert(map_file != NULL);
		printf("load map file fail\n");
	}
    else{
		printf("load map file success\n");
		while(fscanf(map_file,"%s %f %f %f %f %f", &param_name[0], &param[0], &param[1], &param[2], &param[3], &param[4] )!=EOF){
			if(!strcmp(param_name,"Node:")){
				//Listサイズよりデータ数が多い場合は、assert
				if( node_index < LIST_SIZE ){
					Vector2D tmp_v_node(0,0);

					tmp_v_node.x = param[0];
					tmp_v_node.y = param[1];
					NodeList[node_index] = new Node( tmp_v_node, param[2] );
					node_index++;
				}
				else{
					//assertにする;
					printf("node index max NG \n");
				}
			}
			//Line
			else if(!strcmp(param_name,"Line:")){
				//Listサイズよりデータ数が多い場合は、assert
				if( edge_index < LIST_SIZE ){
					Vector2D tmp_v_line(0,0);

					tmp_v_line.x = param[3];
					tmp_v_line.y = param[4];
					EdgeList[edge_index] = new Line( param[0], param[1], param[2], tmp_v_line);
					edge_index++;
				}
				else{
					//assertにする
				}
			}
			//Cricle
			else if(!strcmp(param_name,"Circle:")){
				//Listサイズよりデータ数が多い場合は、assert
				if( edge_index < LIST_SIZE ){
					Vector2D tmp_v_circle(0,0);
					tmp_v_circle.x = param[0];
					tmp_v_circle.y = param[1];

					EdgeList[edge_index] = new Circle( tmp_v_circle, param[2],param[3] );
					edge_index++;
				}else{
					//assertにする
				}
			}

		}
		fclose(map_file);
	}
}

