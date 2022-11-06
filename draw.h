


using namespace std;
using namespace cv;


float B1,B2;
float x_trailer_pivot,y_trailer_pivot;
	
void draw_trailer(Mat draw_im,float B2,float X, float Y);
	
//=====================timer=========================
/*void get_tic()

{
    gettimeofday(&tv,NULL);
    tic = 1000000 * tv.tv_sec + tv.tv_usec;
}

void get_toc()

{
   gettimeofday(&tv,NULL);
   toc = 1000000 * tv.tv_sec + tv.tv_usec;
   float time = (toc - tic)/1000;
   printf("time= %f\n",time);
}
*/

//=====================drawing=======================
void draw_buttons(Mat draw_im){
Point pt1=Point(520,30); // draw rectangles
Point pt2=Point(570,80);
const Scalar& color=Scalar(255,0,0);
int thickness=1;
int lineType=8;
int shift=0;
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(520,30); 
pt2=Point(570,80);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(570,80); // draw rectangle
pt2=Point(620,130);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(520,80); // draw rectangle
pt2=Point(570,130);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(470,80); // draw rectangle
pt2=Point(520,130);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(520,80); // draw rectangle
pt2=Point(570,180);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);
}

void draw_truck(Mat draw_im){
int x=200;//>>>>right shift
int y=-200;//vvvvdown shift
Point pt1=Point(350+x,200+y); // body
Point pt2=Point(475+x,350+y);
const Scalar& color=Scalar(0,255,0);
int thickness=4;
int lineType=8;
int shift=0;
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(335+x,210+y); //left front wheel
pt2=Point(350+x,240+y);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(475+x,210+y); // right front wheel
pt2=Point(490+x,240+y);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(335+x,310+y); // back left wheel
pt2=Point(350+x,340+y);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(475+x,310+y); // back right wheel
pt2=Point(490+x,340+y);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

pt1=Point(408+x,350+y); // draw pivot 412.5+x,354.5 + y pivot coordinate
pt2=Point(417+x,359+y);
rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

/*pt1=Point(412.5+x,100+y);
pt2=Point(412.5+x,600+y);*/

thickness=0.5;

for(int i=260+y ; i<=710+y  ;i=i+20  ){//dashed line
pt1=Point(412.5+x,i+y);
pt2=Point(412.5+x,i+10+y);
line(draw_im, pt1,pt2, color, thickness, lineType, shift);
}

}

void draw_connection(Mat draw_im,float B1,float B2){
int x=200;//>>>>right shift
int y=-200;//vvvvdown shift
	
int a,b;
if(B1<-1.0788 || B1>1.1832){//color changed to red in critical angles
a=0;
b=255;
}else{
a=255;
b=0;
}

const Scalar& color=Scalar(0,a,b);
int thickness=4;
int lineType=8;
int shift=0;
Point pt1,pt2;
	
pt1=Point(412.5+x,354.5+y);//pivot point
pt2=Point(0*cos(B1)-70.5*sin(B1)+412.5+x,0*sin(B1)+70.5*cos(B1)+354.5+ y);//rotated point
x_trailer_pivot=pt2.x;
y_trailer_pivot=pt2.y;

/*if(B1!=0){//draw line
//const Scalar& color=Scalar(255,255,0);*/
line(draw_im, pt1,pt2, color, thickness, lineType, shift);
//}
draw_trailer(draw_im,B2,x_trailer_pivot, y_trailer_pivot);
	
	
}





void draw_trailer(Mat draw_im,float B2,float X, float Y){

	
int a,b;
if(B2<-1.0788 || B2>1.1832){//color changed to red in critical angles
a=0;
b=255;
}else{
a=255;
b=0;
}

int x=200;//>>>>right shift
int y=-200;//vvvvdown shift
X=X-x;
Y=Y-y;
//X,Y will be provided externally
/*float X=412.5;//rotate around this point by B2
float Y=425;*/

const Scalar& color=Scalar(0,a,b);
int thickness=4;
int lineType=8;
int shift=0;
Point pt1,pt2,pt3,pt4;


/*****412.5+x,354.5+y pivot point*****/


//Point pt1=Point(325+x,425+y); // body
//Point pt2=Point(500+x,590+y);           
//rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);
/*pt1=Point((325-X)*cos(B2)-(425-Y)*sin(B2)+X+x,(325-X)*sin(B2)+(425-Y)*cos(B2)+Y+y);//rotated point
pt2=Point((500-X)*cos(B2)-(425-Y)*sin(B2)+X+x,(500-X)*sin(B2)+(425-Y)*cos(B2)+Y+y);//rotated point
pt3=Point((500-X)*cos(B2)-(590-Y)*sin(B2)+X+x,(500-X)*sin(B2)+(590-Y)*cos(B2)+Y+y);//rotated point
pt4=Point((325-X)*cos(B2)-(590-Y)*sin(B2)+X+x,(325-X)*sin(B2)+(590-Y)*cos(B2)+Y+y);//rotated point*/

/*pt1=Point((X-87.5-X)*cos(B2)-(Y-Y)*sin(B2)+X+x,(X-87.5-X)*sin(B2)+(Y-Y)*cos(B2)+Y+y);//rotated point
pt2=Point((X+87.5-X)*cos(B2)-(Y-Y)*sin(B2)+X+x,(X+87.5-X)*sin(B2)+(Y-Y)*cos(B2)+Y+y);//rotated point
pt3=Point((X+87.5-X)*cos(B2)-(Y+165-Y)*sin(B2)+X+x,(X+87.5-X)*sin(B2)+(Y+165-Y)*cos(B2)+Y+y);//rotated point
pt4=Point((X-87.5-X)*cos(B2)-(Y+165-Y)*sin(B2)+X+x,(X-87.5-X)*sin(B2)+(Y+165-Y)*cos(B2)+Y+y);//rotated point
*/
//after simplyfying above we get	
pt1=Point(-87.5*cos(B2)+X+x,-87.5*sin(B2)+Y+y);//rotated point
pt2=Point(87.5*cos(B2)+X+x,87.5*sin(B2)+Y+y);//rotated point
pt3=Point(87.5*cos(B2)-165*sin(B2)+X+x,87.5*sin(B2)+165*cos(B2)+Y+y);//rotated point
pt4=Point(-87.5*cos(B2)-165*sin(B2)+X+x,-87.5*sin(B2)+165*cos(B2)+Y+y);//rotated point	
	
line(draw_im, pt1,pt2, color, thickness, lineType, shift);
line(draw_im, pt2,pt3, color, thickness, lineType, shift);
line(draw_im, pt3,pt4, color, thickness, lineType, shift);
line(draw_im, pt4,pt1, color, thickness, lineType, shift);


//pt1=Point(310+x,520+y); //left  wheel
//pt2=Point(325+x,550+y);
//rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);

/*pt1=Point((310-X)*cos(B2)-(520-Y)*sin(B2)+X+x,(310-X)*sin(B2)+(520-Y)*cos(B2)+Y+y);//rotated point
pt2=Point((325-X)*cos(B2)-(520-Y)*sin(B2)+X+x,(325-X)*sin(B2)+(520-Y)*cos(B2)+Y+y);//rotated point
pt3=Point((325-X)*cos(B2)-(550-Y)*sin(B2)+X+x,(325-X)*sin(B2)+(550-Y)*cos(B2)+Y+y);//rotated point
pt4=Point((310-X)*cos(B2)-(550-Y)*sin(B2)+X+x,(310-X)*sin(B2)+(550-Y)*cos(B2)+Y+y);//rotated point*/
	
pt1=Point(-102.5*cos(B2)-95*sin(B2)+X+x,-102.5*sin(B2)+95*cos(B2)+Y+y);//rotated point
pt2=Point(-87.5*cos(B2)-95*sin(B2)+X+x,-87.5*sin(B2)+95*cos(B2)+Y+y);//rotated point
pt3=Point(-87.5*cos(B2)-125*sin(B2)+X+x,-87.5*sin(B2)+125*cos(B2)+Y+y);//rotated point
pt4=Point(-102.5*cos(B2)-125*sin(B2)+X+x,-102.5*sin(B2)+125*cos(B2)+Y+y);//rotated point	
	

	
	
line(draw_im, pt1,pt2, color, thickness, lineType, shift);
line(draw_im, pt2,pt3, color, thickness, lineType, shift);
line(draw_im, pt3,pt4, color, thickness, lineType, shift);
line(draw_im, pt4,pt1, color, thickness, lineType, shift);


//pt1=Point(500+x,520+y); // right  wheel
//pt2=Point(515+x,550+y);
//rectangle(draw_im, pt1, pt2, color, thickness, lineType, shift);
/*pt1=Point((500-X)*cos(B2)-(520-Y)*sin(B2)+X+x,(500-X)*sin(B2)+(520-Y)*cos(B2)+Y+y);//rotated point
pt2=Point((515-X)*cos(B2)-(520-Y)*sin(B2)+X+x,(515-X)*sin(B2)+(520-Y)*cos(B2)+Y+y);//rotated point
pt3=Point((515-X)*cos(B2)-(550-Y)*sin(B2)+X+x,(515-X)*sin(B2)+(550-Y)*cos(B2)+Y+y);//rotated point
pt4=Point((500-X)*cos(B2)-(550-Y)*sin(B2)+X+x,(500-X)*sin(B2)+(550-Y)*cos(B2)+Y+y);//rotated point*/
	
pt1=Point(102.5*cos(B2)-95*sin(B2)+X+x,102.5*sin(B2)+95*cos(B2)+Y+y);//rotated point
pt2=Point(87.5*cos(B2)-95*sin(B2)+X+x,87.5*sin(B2)+95*cos(B2)+Y+y);//rotated point
pt3=Point(87.5*cos(B2)-125*sin(B2)+X+x,87.5*sin(B2)+125*cos(B2)+Y+y);//rotated point
pt4=Point(102.5*cos(B2)-125*sin(B2)+X+x,102.5*sin(B2)+125*cos(B2)+Y+y);//rotated point	
line(draw_im, pt1,pt2, color, thickness, lineType, shift);
line(draw_im, pt2,pt3, color, thickness, lineType, shift);
line(draw_im, pt3,pt4, color, thickness, lineType, shift);
line(draw_im, pt4,pt1, color, thickness, lineType, shift);



pt1=Point(X+x,Y+y);//pivot point
pt2=Point(0*cos(B2)-200*sin(B2)+X+x,0*sin(B2)+200*cos(B2)+Y+y);//rotated point


if(B2!=0){//draw line
const Scalar& color2=Scalar(255,255,0);
line(draw_im, pt1,pt2, color2, thickness, lineType, shift);
}


}




void draw_guide(Mat draw_im,float B1, float B2){

int n=500;
float k=500;
const Scalar& color=Scalar(0,255,100);
int thickness=1;
int lineType=8;
int shift=0;
Point pt1,pt2,pt3,pt4;


pt1=Point(100,435);
pt2=Point(1180,435);

line(draw_im, pt1,pt2, color, thickness, lineType, shift);//draw horizontal line

//draw triangle - actual steer position - use ADC data to get n 
Point triangle_points[1][3];
triangle_points[0][0]=Point(n,435);
triangle_points[0][1]=Point(n-20,460);
triangle_points[0][2]=Point(n+20,460);
const Point* ppt[1] = { triangle_points[0] };
int npt[] = { 3 };
fillPoly( draw_im,ppt,npt,1,Scalar( 200,20, 0 ),lineType );

//draw triangle - desired steer position
	k=6.773*B1-6.3263*B2; //calculate desired angle using angle data
	k=k*57.2958;//convert to deg
	if(k>45){
		k=1180;//k=45;
	}else if(k<-45){
		k=100;//k=-45;
	}else if(k==0){
		k=540;
	}else if(k>0){
		k=540+(540/45)*k;
	}else if(k<0){
	    k=540-(440/45)*(-k);
		}
	
	
//Point triangle_points[1][3];
triangle_points[0][0]=Point(k,435);
triangle_points[0][1]=Point(k-80,400);
triangle_points[0][2]=Point(k+80,400);
ppt[1] = { triangle_points[0] };

fillPoly( draw_im,ppt,npt,1,Scalar( 0,255, 0 ),lineType );
	



}
