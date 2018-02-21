#include "darknet.h"
#include "network.h"
#include "box.h"
#include "image.h"

#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

typedef struct adjBox
{
	int x;
	int y;
	int w;
	int h;
	int classID;  // jb  

} adjBox;

typedef struct boxInfo
{
	adjBox* boxes;
	int num;
	//IplImage* im;
} boxInfo;

// Hack: adding _obj to avoid name collision
// 
box* init_boxes_obj(network* net);
float** init_probs_obj(network* net);

image **load_alphabet_obj_(char* path);
void extratObject(int imW, int imH, int num, float thresh, box *boxes, float **probs, char **names, int classes, boxInfo *result);

void run_yolo_detection_obj(image im, network *net, box *boxes, float **probs, float thresh, float hier_thresh, char **names, boxInfo *result);

