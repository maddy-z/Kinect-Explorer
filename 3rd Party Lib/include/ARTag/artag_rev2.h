//ARTag Rev 2k:  planar array and OpenGL support version - last revision Nov 2006
//
//May 30/2006 - setting switches changed and version string added
//Oct 4/2005 by Mark Fiala, National Research Council of Canada
//       email: mark.fiala@nrc-cnrc.gc.ca
//
//-free for non-commercial uses such as research and education
//-this evaluation version times out after June 30/2007
//- newer evaluation versions will be periodically available from NRC/IIT website
//- check website www.artag.net for possible updated info not on NRC/IIT website

//-call init_artag() with the input image dimensions before calling any other functions
//-width,height are dimensions of images passed in detect functions
//-bpp=bytes per pixel=colour depth, bpp is only used by drop-in function artagDetectMarker()
// bpp=1 for greyscale 8-bit, 3=24-bit, 4=32-bit input images
char  init_artag(int width, int height, int bpp);
void  close_artag(void);


//--------------------------- ARTag Objects (new in Rev2) -------------------------------
//-an object is either a single ARTag marker, or a 2D array
//-perform the following steps 1,2,3 once when initializing, then performs steps
//every image frame
//---- initialization -----
//-1-first call init_artag()
//-2-then load an array (.cf) file with load_array_file() if you want to use arrays
//-3-associate single markers or arrays with objects, use returned artag_object_id
//   for future calls
//---- every frame -----
//-4-call artag_find_objects();  this searches the image for objects
//-5-for all objects associated in step 3, call artag_is_object_found(obj#), if
//   result is true (1) then perform steps
//-6-call artag_set_object_opengl_matrix(), which sets modelview matrix for you to start rendering
//-7-other calls you could make instead of (or as well as) artag_set_object_opengl_matrix()
//   are artag_project_between_objects() for transferring object coordinates (see artag_cad.cpp example program)


int load_array_file(char *filename);   //returns -1 if file not found

//-associate an array with an object, this function will return an ID to use in future
//calls.  "frame_name" is the same as in the array .cf file which must be loaded first.
//-if the return value is -1, the object could not be initialized
//example: base_artag_object_id=artag_associate_array("base0");
//NOTE: if you make your own arrays, each artag_id can only be used ONCE otherwise array
//      object pose will be erroneously detected (no outlier rejection implemented thus far)
int artag_associate_array(char *frame_name);  //returns artag_object_id

//-associate an object with a single ARTag marker
//-dimensions are 0,0 to 1,1 if artag_get_object_coordframe_range() is called
//-if the return value is -1, the object could not be initialized
//example: demo_object_id=artag_associate_marker(1023);  //associate with ARTag ID 1023
//NOTE: artag_id cannot be in any arrays declared in array file, arrays take precendence,
//      plus if array containing this ID is also visible, it will highly degrade array object
//      pose
int artag_associate_marker(int artag_id);  //returns artag_object_id

//-artag_find_objects() is the main call that does all the work, it searches the image
//provided for markers, and then finds those that belong to defined objects
//-rgb_greybar =1 for RGB images, =0 for greyscale
//-artag_find_objects() internally calls the artag_find_marker() function from Rev1
//-artag_find_objects() returns the number of objects found in the image for diagnostic 
//purposes
int artag_find_objects(unsigned char *rgb_cam_image, char rgb_greybar);

//-artag_is_object_found(artag_object_num) returns 1 if object was found from most
//recent artag_find_objects() call, returns 0 if object was not found 
char artag_is_object_found(int artag_object_num);

//-find OpenGL modelview matrix, example code at bottom of this section
//-only call this if artag_is_object_found(artag_object_num) returned 1 in this image frame
//-mirror_on =0 for normal case, mirror_on=1 for horizontal mirroring such as "magic mirror"
//applications
void artag_set_object_opengl_matrix(int object_num, char mirror_on);


//--- Example usage with OpenGL ---
//if(artag_is_object_found(base_artag_object_id)) 
//	{
//	float opengl_matrix[16];
//	artag_set_object_opengl_matrix(base_artag_object_id,opengl_matrix);
//  glBegin(GL_TRIANGLES);
//  ...

//------------------------- REV2 Support Functions -------------------------------
//artag_set_camera_params() - call this function if you are doing 3D augmentations
//                            or pose determination, ignore otherwise
//-these 4 parameters make up the 'K' matrix, from camera calibration
//-you can quickly calibrate your camera manually with a ruler or tape measure
//by finding the image width 'u' of an object IN THE CAMERA! (not screen) image of an 
//object of known width 'w' at a known distance 'd'.  
//then, camera_fx = u * d / w
//-in most cases you can assume square pixels (not true for NTSC/PAL/SECAM input, depends on frame-grabber)
//and can set camera_fy=camera_fx
//-if calibrating manually, assume camera_cx,camera_cy are the center of the camera image,
//i.e. for a 640x480 input image, camera_cx=320 ,camera_cy -240
void artag_set_camera_params(double camera_fx, double camera_fy,
                             double camera_cx, double camera_cy);


//-find range of marker world coordinates for elements of objects, useful for drawing
//boxes or grids, or resizing 3D meshes to fit
//-example: artag_get_object_coordframe_range(&min_x,&max_x,&min_y,&max_y,&min_z,&max_z);
//-for arrays, it returns the minimum and maximum X & Y values
//-for objects assigned to individual ARTag markers, it will return 0,1,0,1
//-min_z,max_z will always be zero for ARTag Rev2
void artag_get_object_coordframe_range(int object_num,
                                      float *min_x, float *max_x,
                                      float *min_y, float *max_y,
                                      float *min_z, float *max_z);

//misspelled version of above also provided, to match with previous ARTag releases that had "artag_" spelled "atag_"
void atag_get_object_coordframe_range(int object_num,
                                      float *min_x, float *max_x,
                                      float *min_y, float *max_y,
                                      float *min_z, float *max_z);


//-find what an X,Y,Z in an object maps to in camera image coordinates (NOT! screen
//coordinates if the screen is a different resolution as typically with OpenGL)
//-only call this if artag_is_object_found(artag_object_num) returned 1 in this image frame
void artag_project_point(int object_num, float x, float y, float z, float *u, float *v);


//-find what X,Y,Z in one object maps to in another, useful for interactivity and
//detecting collisions between objects
//-only call this if artag_is_object_found(artag_object_num) returned 1 in this image frame
void artag_project_between_objects(int object_num1,double source2dest_scale,
                                   int object_num2,
                                   double x1, double y1, double z1,
								   double *x2, double *y2, double *z2);

//-remove rotation of selected object, so X,Y,Z axis are aligned with virtual camera, useful
//for effects where you want a virtual object to move but not rotate, such as fire effects,
//"cardboard" cutout effects, etc.  
//Call artag_remove_object_rotation() before artag_set_object_opengl_matrix().
void artag_remove_object_rotation(int object_num);

//-find out whether object is a single marker, array, (or future expansion types)
//type=artag_get_object_type(object_num);
//0=single marker
//1=normal coordframe
int artag_get_object_type(int object_num);

//-------------------------- Choosing ARTag Markers for Application ----------------------------
//call this function repeatedly to get markers in order off of recommended list
//add 1024 to result if you want white-on-black markers
int artag_get_id(int width, int height, char *name);
//width,height are ignored, they are set by init_artag().

//-artag_get_id() pulls markers off of artag_recommended[] and artag_recommended_hd[] static arrays
//-artag_recommended_marker_pointer is the global that is incremented apon every call.  set to 0
//to start again in the artag_recommended[] list

//-------------------------- Creating ARTag Marker Patterns ----------------------------
//-Call artag_create_marker() to create a bitmap of (10*scale) x (10*scale) bytes
//artag_create_marker() will fill an unsigned char array with 100*scale*scale bytes
//
//if(artag_create_marker(567,5,image)) problem...  
//create ARTag marker ID=567, 5 pixels/bit, total image will be 50x50 pixels
//this function will malloc room if image is NULL
//returns -1 if problem, 0 otherwise

int  artag_create_marker(int artag_id, int scale, unsigned char *image);

//NOTE: If you create your own array files, only use each artag_id once!  Otherwise pose 
//      determination will be erratic and not work for those objects.  Also you cannot repeat
//      a marker between an array and an individual marker.  It's best to choose marker ID's
//      sequentially, or to keep calling artag_get_id() so that you don't repeat markers



//-------------- Adjusting ARTag Performance Switches -------------
//In ARTag Rev 1&2 there are 4 settings you can change to affect processing time and to
//active/deactive heuristics that affect quadrilateral border detection.  Below are 8 functions
//that allow you to do this:

//-turn off full resolution processing to reduce processing time by about 3 times
//half and quarter resolution is always processed, this setting=1 processes the full resolution
//as well
//1=use full resolution as well as half, quarter resolution  (default state)
//0=only process half, quarter resolution  (faster but smaller markers will be missed)
char artag_get_setting_use_full_res(void);
void artag_set_setting_use_full_res(char setting);

//broken border heuristics can repair broken marker border
// 1=enable/0=disable repairing incomplete marker borders (default=1)
char artag_get_setting_repair_broken_border(void);
void artag_set_setting_repair_broken_border(char setting);

//artag_set_switch() function from previous versions removed, performs the same as above
//some of the old switches had problems and are now removed


//-------------- Read version information -------------
void artag_get_version(char *str);

//-------------- Debugging -------------
//-if artag_find_objects() does not seem to be finding anything, there could be a problem in
//the image such as incorrectly copied/loaded data, incorrectly set width,height,or bpp from when
//init_artag() was called, or a wrong rgb_greybar setting in if artag_find_objects().  A test
//mode can be turned on which writes out a .PGM file of what artag_find_objects() is actually
//processing.  This is good to verify that you are indeed giving ARTag the image you think you
//are.  This is a good sanity check of the system.
//-an internal flag (global variable) inside the ARTag library (called 'output_image_mode')
//determines if <artag_debug_image.pgm> is written out every time artag_find_objects() is called.
//-'output_image_mode' is set by artag_set_output_image_mode() and cleared by
//artag_clear_output_image_mode().  
//-call artag_set_output_image_mode() some time AFTER you call artag_init() since artag_init()
//clears this to the default cleared (0) state.
//-NOTE: if you call artag_set_output_image_mode() but don't clear it, it will write out
//the debug image *every* image frame which will slow things down, so remember to clear it or
//remove it from your program.
//-a greyscale debug image will be written out regardless of if the input image is RGB or
//greyscale.
void artag_set_output_image_mode(void);     //turn on output debug image writing 
void artag_clear_output_image_mode(void);   //turn off output debug image writing





