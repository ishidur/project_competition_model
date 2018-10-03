#include "VirtualMap.h"
#include "Positioning/MapInfomation/Line.h"
#include "Positioning/MapInfomation/Circle.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ev3api.h"

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
	float param[6];

	// course
	char course;

	FILE* course_file = fopen("/ev3rt/res/course/course.txt","r");
	if(course_file==NULL){
		printf("load course file fail\n");
		assert(course_file != NULL);
	}

	char course_param_name[255] = {'\0'};
	fscanf(course_file,"%s %c", &course_param_name[0], &course);
	fclose(course_file);
    printf("load course: %c\n", course);

    //map
    FILE* map_file = NULL;

    //R
    if( course == 'R' ){
    	map_file = fopen("/ev3rt/res/navigation/map_params_R.txt","r");
    }
    //L
    else{
    	map_file = fopen("/ev3rt/res/navigation/map_params_L.txt","r");
    }

	if( map_file == NULL ){
		printf("load map file fail\n");
		assert(map_file != NULL);
	}
    else{
		printf("load map file success\n");
		while(fscanf(map_file,"%s %f %f %f %f %f %f", &param_name[0], &param[0], &param[1], &param[2], &param[3], &param[4], &param[5] )!=EOF){
			//Node
			if(!strcmp(param_name,"Node:")){
				//Listサイズよりデータ数が多い場合は、assert
				if( node_index < LIST_SIZE ){
					Vector2D tmp_v_pos( param[0] ,param[1] );
					Vector2D tmp_v_direction( param[2] ,param[3] );

					NodeList[node_index] = new Node( tmp_v_pos, tmp_v_direction, param[4] );
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
					Vector2D tmp_v_line( param[3], param[4] );

					EdgeList[edge_index] = new Line( param[0], param[1], param[2], tmp_v_line, param[5] );
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
					Vector2D tmp_v_circle( param[0], param[1]);

					EdgeList[edge_index] = new Circle( tmp_v_circle, param[2], param[3], param[4] );
					edge_index++;
				}else{
					//assertにする
				}
			}

		}
		fclose(map_file);
	}
}