/*
   Interface between Matheval and the distribution library.
   These are functions that need to be C call-able, as matheval is C.
   However, the functions need to be compiled in C++ as the distribution 
   library (and Eigen) needs c++.
   So the interface functions take void pointers that are then cast to C++
   objects. The C code only sees these as void pointers and does not interpret them.

   dml 6/6/12 c 2012 Fordham RCVLab 
 */
/*  Include file for distribution libraryu*/

#include <iostream>
#include<string>
#include "problem.h"
#include "array.h"
#include <string.h>
#include <math.h>


#include "amcl.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
geometry_msgs::PoseWithCovarianceStamped amcl_pose;

using namespace std;

#define SIGN(x) ( ((x)>=0)?(1.0):(-1.0) )
#define MIN(x,y) ( ((x)>(y))?(y):(x) )

string ftoa(double x) {
  stringstream ss;
  ss<<x;
  return ss.str();
}

void checkpoint()
{
	bool debug_mode = false;
	if (debug_mode == false){
	}
	else{
		int c;
		printf( "Press ENTER to continue... " );
		fflush( stdout );
		do c = getchar(); while ((c != '\n') && (c != EOF));
	}
}

MoG * invDist(MoG* d1)
{
    //  (1 / distribution) to figure out the magnitude of repulsion
    // which is larger when closer / distribution is smaller
    string name = "(1/" + d1->distribution_name + ")";
    MoG temp(name, d1->peak.size());
    
    for(int i=0;i<d1->peak.size();i++){
      temp.peak[i] = d1->peak[i];
      double tempTotal=0;
      for(int j=0;j<d1->peak[i].mean.size();j++){
        if(d1->peak[i].mean[j]==0)
          tempTotal += 0;
        else
          tempTotal += (1/d1->peak[i].mean[j]);
        }
      for(int j=0;j<temp.peak[i].mean.size();j++){
        temp.peak[i].mean[j] = tempTotal;
	//careful .. depending on the use of invdist, you may want result to be positive
	//ie a scaling factor for some expression should be sign neutral to let e's sign dominate -dgn
      }
    }
    
    temp.check();
    temp.print();
    MoG *pNew = new MoG();
    *pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
    return pNew;
}

// superpose ADDS two distributions together to form a single normalized distribution
// the peaks for each distribution are added to the final distribution
// the weights are adjusted so that d2's weights are subtracted from d1's weights before normalize

MoG * superpose(MoG* d1, MoG* d2)
{
    string name = "(superpose " + d1->distribution_name + "+" + d2->distribution_name + ")";

    int peakSize = d1->peak.size() + d2->peak.size();
    
    cout << "Inputs to superpose:\n";
    d1->print();
    d2->print();
    cout << "d1.size() = " << d1->peak.size() << "  d2.size() = " << d2->peak.size() << endl;
    MoG temp(name, peakSize);
    
    for(int i=0;i<d1->peak.size();i++){//copy d1 (v1C) in multirobot example
      temp.peak[i]=d1->peak[i];
    }
    for(int j=0;j<d2->peak.size();j++){//for every obstacle intersection
      float newWeight=d2->peak[j].weight;
      // find corresponding peak
      temp.peak[j+d1->peak.size()]=d2->peak[j]; //keep the obstacle intersection in every d1.size()th location

      for(int i=0;i<d1->peak.size();i++){
        if(temp.peak[i].peakID == d2->peak[j].peakID){
          // remove that portion from peak[i]
	  cout<<"Superpose result/d1 peak "<<i<<" weight adjusted by result/d2 peak "<<j<<endl;
          cout<<"  was "<<temp.peak[i].weight;

	  temp.peak[i].weight = d1->peak[i].weight - d2->peak[j].weight*d1->peak[i].weight;
	  //temp.peak[j+d1->peak.size()].weight = d2->peak[j].weight*d1->peak[i].weight;
	  newWeight=d2->peak[j].weight*d1->peak[i].weight;
          cout<<" now is  "<<temp.peak[i].weight<<"(- "<<d2->peak[j].weight<<")\n";
          temp.peak[j+d1->peak.size()].mean = d2->peak[j].mean + d1->peak[i].mean;
          temp.peak[j+d1->peak.size()].variance = d2->peak[j].variance + d1->peak[i].variance;
        }// if color0==color1
      } //for i
      cout<<"and new weight for result/d2 peak "<<j+d1->peak.size()<<" is "<<newWeight<<endl;
      temp.peak[j+d1->peak.size()].weight=newWeight; //keep the obstacle intersection in every d1.size()th location

    }// for j
    cout<<"Superpose uncleaned result\n";
    temp.print();

    cout << "Cleaning up after superpose, generating NO COLORS EVER\n";
    temp.check();
    // colors here cause problems for ConvolveMM later
    // if (!temp.isColored()) temp.color(); // superpose generates colors IFF no colors present, dml 2/14,15/14
    temp.print();

    MoG *pNew = new MoG();
    *pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
    return pNew;
}

MoG *CondConvolve(MoG d1, MoG d2)
{
        //d1.color();
        //d2.color();
	string name = "(" + d1.distribution_name + "*" + d2.distribution_name + ")";
	MoG temp(name,d1.peak.size());
/** Now there are three cases:
 * case 1: cond * cond
 * 		-> Take LHS result peak and apply to RHS evaluation
 * case 2: RHS is freezer
 * 		-> pre-freeze the MoG, and freeze permanently after the reverse-step is applied
 **/
// First consider case 1
	if(d1.peak.size()!=d2.peak.size()){
		printf ("LHS peak# =/= RHS peak#. \n");
		cout << name << endl;
		exit(1);
	}
// Freeze check
//	else{
	for(int i=0; i<d1.peak.size();i++){
		temp.peak[i] = d1.peak[i];

		if (d1.peak[i].freeze==2){
			// frozen, do nothing
		}
		else if(d2.peak[i].freeze==1){
			temp.peak[i].freeze = 1;// apply pre-freezer
			int d1condOp = d1.condOp;
			Vector * d1bound = d1.condBound;
			double Wall_2_x = d1bound->point[0].value[0];
			double Wall_2_y = d1bound->point[0].value[1];
			int pass=0;
			int n = 4000;
			double value_Wall_2;
			MatrixXd tempPts = d1.mvnrng(i,n);
			MatrixXd result_matrix;
			// case 1: GTR wall_2
			if(d1condOp==1){
				for(int j=0;j<n;j++){
					value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
					if(value_Wall_2>0){
						pass++;
						result_matrix.conservativeResize(pass,2);
						result_matrix(pass-1,0)=tempPts(j,0);
						result_matrix(pass-1,1)=tempPts(j,1);
					}
				}
			}					
			// case 2: LTR wall_2
			else if(d1condOp==2){
				for(int j=0;j<n;j++){
					value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
					if(value_Wall_2<0){
						pass++;
						result_matrix.conservativeResize(pass,2);
						result_matrix(pass-1,0)=tempPts(j,0);
						result_matrix(pass-1,1)=tempPts(j,1);
					}
				}
			}
			else{
				cout << "Op case not found...please doublecheck pars file.\n";
				d1.print();
				d2.print();
				exit(1);
			}
			double P=pass+0.00000001;
			double N=n+0.01;
			double result = P/N;
			if (result <= 0.005)
				result = 0;
			if (result >= 0.995)
				result = 1;	
			double xsum=0,ysum=0, xvar=0, yvar=0, covar=0;
			for(int j=0;j<pass;j++){
				// this part takes sum; should move to GTR/LTR loop later
			    xsum+=result_matrix(j,0);
			    ysum+=result_matrix(j,1);
			}
			double xmean=xsum/P;
			double ymean=ysum/P;
			for(int j=0;j<pass;j++){
				xvar+=pow((result_matrix(j,0)-xmean),2);
				yvar+=pow((result_matrix(j,1)-ymean),2);
				covar+=((result_matrix(j,0)-xmean)*(result_matrix(j,1)-ymean));
			}
			xvar = xvar/(P-1);
			yvar = yvar/(P-1);
			covar = covar/(P-1);
			temp.peak[i].weight = result * temp.peak[i].weight;
			temp.peak[i].mean.conservativeResize(2);
			temp.peak[i].mean[0]=xmean;
			temp.peak[i].mean[1]=ymean;
			temp.peak[i].variance.conservativeResize(2,2);
			temp.peak[i].variance(0,0) = xvar;
			temp.peak[i].variance(0,1) = covar;
			temp.peak[i].variance(1,0) = covar;
			temp.peak[i].variance(1,1) = yvar;
//				temp.peak[i].variance = (result) * temp.peak[i].variance;
//			temp.condOp = d2condOp;
//			temp.condBound = d2bound;
			// move to the closest wall
			/*
			Vector * d1bound = d1.condBound;
			Vector * d2bound = d2.condBound;
			double Wall_1_x = d1bound->point[0].value[0];
			double Wall_1_y = d1bound->point[0].value[1];
			double Wall_2_x = d2bound->point[0].value[0];
			double Wall_2_y = d2bound->point[0].value[1];
			double slope_1 = Wall_1_y / Wall_1_x;
			double slope_2 = Wall_2_y / Wall_2_x;
			double dist_wall_1=abs(Wall_1_x*d1.peak[i].mean[0]+Wall_1_y*d1.peak[i].mean[1]-(Wall_1_x*Wall_1_x+Wall_1_y*Wall_1_y))/sqrt(Wall_1_x*Wall_1_x+Wall_1_y*Wall_1_y);
			double dist_wall_2=abs(Wall_2_x*d1.peak[i].mean[0]+Wall_2_y*d1.peak[i].mean[1]-(Wall_2_x*Wall_2_x+Wall_2_y*Wall_2_y))/sqrt(Wall_2_x*Wall_2_x+Wall_2_y*Wall_2_y);
			double offset, angle;
			if (dist_wall_1>dist_wall_2){
				offset = dist_wall_2;
				angle = atan(slope_2);
		}
			else{
				offset = dist_wall_1;
				angle = atan(slope_1);
			}
//			double slope = b/a;
//			double offset = (a*mu[0]+b*mu[1]-(a*a+b*b))/(a+b*slope);
			d1.peak[i].mean[0] += (offset)*cos(angle);
			d2.peak[i].mean[1] += (offset)*sin(angle);
			*/
		}
		else if (d1.peak[i].freeze==0/* && d2.peak[i].freeze==0*/){
/** HERE WE DO THE TRICK!!
 * 1. Grab the operation code from both side	->condOp, 1:GTR, 2:LTR
 * 2. Grab the line from both side	->condBound
 * 3. Generate a lot of points	->MatrixXd temp = mvnrng(i,n); i=peak#, n=rng size
 * 4. see how many points falls into the region
 * 	- GTR*GTR, GTR*LTR, LTR*GTR, LTR*LTR
 * 5. Approaches the points generated with a normal distribution, Xmean and Ymean, Xvar and Yvar.
 * 6. PROBLEM!! How to set the result to fit 3COND cases? 
 * 	- Just assign one of OP and LINE to it, it will fit anyway*/
			int d1condOp = d1.condOp, d2condOp = d2.condOp;
			Vector * d1bound = d1.condBound;
			Vector * d2bound = d2.condBound;
			double Wall_1_x = d1bound->point[0].value[0];
			double Wall_1_y = d1bound->point[0].value[1];
			double Wall_2_x = d2bound->point[0].value[0];
			double Wall_2_y = d2bound->point[0].value[1];
			int pass=0;
			int n = 4000;
			double value_Wall_1, value_Wall_2;
			MatrixXd tempPts = d1.mvnrng(i,n);
			MatrixXd result_matrix;
			// case 1: GTR wall_1 and GTR wall_2
			if(d1condOp==1&&d2condOp==1){
				for(int j=0;j<n;j++){
					value_Wall_1=Wall_1_x*tempPts(j,0)+Wall_1_y*tempPts(j,1)-(pow(Wall_1_x,2)+pow(Wall_1_y,2));
					value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
					if(value_Wall_1>0 && value_Wall_2>0){
						pass++;
						result_matrix.conservativeResize(pass,2);
						result_matrix(pass-1,0)=tempPts(j,0);
						result_matrix(pass-1,1)=tempPts(j,1);
					}
				}
			}					
			// case 2: GTR wall_1 and LTR wall_2
			else if(d1condOp==1&&d2condOp==2){
				for(int j=0;j<n;j++){
					value_Wall_1=Wall_1_x*tempPts(j,0)+Wall_1_y*tempPts(j,1)-(pow(Wall_1_x,2)+pow(Wall_1_y,2));
					value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
					if(value_Wall_1>0 && value_Wall_2<0){
						pass++;
						result_matrix.conservativeResize(pass,2);
						result_matrix(pass-1,0)=tempPts(j,0);
						result_matrix(pass-1,1)=tempPts(j,1);
					}
				}
			}
			// case 3: LTR wall_1 and GTR wall_2
			else if(d1condOp==2&&d2condOp==1){
				for(int j=0;j<n;j++){
					value_Wall_1=Wall_1_x*tempPts(j,0)+Wall_1_y*tempPts(j,1)-(pow(Wall_1_x,2)+pow(Wall_1_y,2));
					value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
					if(value_Wall_1<0 && value_Wall_2>0){
						pass++;
						result_matrix.conservativeResize(pass,2);
						result_matrix(pass-1,0)=tempPts(j,0);
						result_matrix(pass-1,1)=tempPts(j,1);
					}
				}
			}
			// case 4; LTR wall_1 and LTR wall_2
			else if(d1condOp==2&&d2condOp==2){
				for(int j=0;j<n;j++){
					value_Wall_1=Wall_1_x*tempPts(j,0)+Wall_1_y*tempPts(j,1)-(pow(Wall_1_x,2)+pow(Wall_1_y,2));
					value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
					if(value_Wall_1<0 && value_Wall_2<0){
						pass++;
						result_matrix.conservativeResize(pass,2);
						result_matrix(pass-1,0)=tempPts(j,0);
						result_matrix(pass-1,1)=tempPts(j,1);
					}
				}
			}
			else{
				cout << "Op case not found...please doublecheck pars file.\n";
				d1.print();
				d2.print();
				exit(1);
			}
			double P=pass+0.00000001;
			double N=n+0.01;
			double result = P/N;
			if (result <= 0.005)
				result = 0;
			if (result >= 0.995)
				result = 1;	
			double xsum=0,ysum=0, xvar=0, yvar=0, covar=0;
			for(int j=0;j<pass;j++){
				// this part takes sum; should move to GTR/LTR loop later
			    xsum+=result_matrix(j,0);
			    ysum+=result_matrix(j,1);
			}
			double xmean=xsum/P;
			double ymean=ysum/P;
			for(int j=0;j<pass;j++){
				xvar+=pow((result_matrix(j,0)-xmean),2);
				yvar+=pow((result_matrix(j,1)-ymean),2);
				covar+=((result_matrix(j,0)-xmean)*(result_matrix(j,1)-ymean));
			}
			xvar = xvar/(P-1);
			yvar = yvar/(P-1);
			covar = covar/(P-1);
			temp.peak[i].weight = result * temp.peak[i].weight;
			temp.peak[i].mean.conservativeResize(2);
			temp.peak[i].mean[0]=xmean;
			temp.peak[i].mean[1]=ymean;
			temp.peak[i].variance.conservativeResize(2,2);
			temp.peak[i].variance(0,0) = xvar;
			temp.peak[i].variance(0,1) = covar;
			temp.peak[i].variance(1,0) = covar;
			temp.peak[i].variance(1,1) = yvar;
//				temp.peak[i].variance = (result) * temp.peak[i].variance;
			temp.condOp = 0;//d2condOp;
			temp.condBound = d2bound;
		}
		else{
			cout << "ERROR in Cond*Cond!! Case not found.\n";
			exit(1);
		}
	}
	temp.check();
	cout<<"TSS; dlibinterface; condconvolve originating colors!\n";
	//temp.color();
	temp.print();
	MoG *pNew = new MoG();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

MoG * convolveMZ(MoG d1, MoG d2)
{
  cout << "ConvolveMZ:\n";
  int num_peak = d1.peak.size() * d2.peak.size();
  string result_name = "(" + d1.distribution_name + "*" + d2.distribution_name + ")";
  MoG temp(result_name, num_peak);
 //	string name = d1.distribution_name + "*" + d2.distribution_name;
 //	MoG temp(result_name,d1.peak.size());
/** Now there are three cases:
 * case 1: cond * cond
 * 		-> Take LHS result peak and apply to RHS evaluation
 * case 2: RHS is freezer
 * 		-> pre-freeze the MoG, and freeze permanently after the reverse-step is applied
 **/
  for(int i=0; i<d1.peak.size();i++){
    for (int j=0;j<d2.peak.size();j++){
      int tempPeakNum = j+i*d2.peak.size();
      if (d1.peak[i].freeze==2){
        cout << "d1.peak" << i << " frozen.\n";
	temp.peak[tempPeakNum] = d1.peak[i];
	temp.peak[tempPeakNum].peakID = j;
	temp.peak[tempPeakNum].weight = 0;
	// frozen, do nothing just copy
      }
      else if(d2.peak[j].freeze==2){
	temp.peak[tempPeakNum] = d1.peak[i];
	temp.peak[tempPeakNum].peakID = j;
	temp.peak[tempPeakNum].weight = 0;
	// frozen, do nothing just copy
      }
      else if(d2.peak[j].freeze==1){
        cout << "d2.peak" << j << " pre-frozen, freezing this peak.\n";
	temp.peak[tempPeakNum] = d2.peak[j];
	//temp.peak[tempPeakNum] = d1.peak[i];
	temp.peak[tempPeakNum].freeze = 2;
	temp.peak[tempPeakNum].peakID = j;
	temp.peak[tempPeakNum].weight = d1.peak[i].weight *d2.peak[j].weight;
      }
      else if (d1.peak[i].freeze==0 && d2.peak[j].freeze==0){
        cout << "d1.peak" << i << " & d2.peak" << j << " not frozen, convolving w/ color.\n";
	temp.peak[tempPeakNum] = d1.peak[i];
        temp.peak[tempPeakNum].peakID = j;
        //temp.peak[tempPeakNum].weight = d1.peak[i].weight *d2.peak[j].weight;
	if(d2.condOp<100){
          int d2condOp = d2.condOp;
          Vector * d2bound = d2.condBound;
          double Wall_2_x = d2bound->point[0].value[0];
          double Wall_2_y = d2bound->point[0].value[1];
          int pass=0;
          int n = 4000;
          double value_Wall_2;
          MatrixXd tempPts = d2.mvnrng(j,n);
          MatrixXd result_matrix;
          // case 1: GTR wall_2
          if(d2condOp==1){
	    for(int j=0;j<n;j++){
	      value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
	      if(value_Wall_2>0){
		pass++;
		result_matrix.conservativeResize(pass,2);
		result_matrix(pass-1,0)=tempPts(j,0);
		result_matrix(pass-1,1)=tempPts(j,1);
	      }
	    }
          }
          // case 2: LTR wall_2
          else if(d2condOp==2){
	    for(int j=0;j<n;j++){
	      value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
	      if(value_Wall_2<0){
		pass++;
		result_matrix.conservativeResize(pass,2);
		result_matrix(pass-1,0)=tempPts(j,0);
		result_matrix(pass-1,1)=tempPts(j,1);
	      }
	    }
          }
          else if(d2condOp==0){
	    temp.peak[tempPeakNum].weight = d1.peak[i].weight *d2.peak[j].weight;
	    temp.peak[tempPeakNum].peakID = d2.peak[j].peakID;
          }
          else{//if it's less than 100 but not equal to 0, 1, 2  !?!?!
	    cout << "Op case not found...please doublecheck pars file.\n";
	    d1.print();
	    d2.print();
	    exit(1);
          }
          if(d2condOp!=0){
	    double P=pass+0.00000001;
	    double N=n+0.01;
	    double result = P/N;
	    if (result <= 0.005)
	      result = 0;
	    if (result >= 0.995)
	      result = 1;	
	    /*
	      double xsum=0,ysum=0, xvar=0, yvar=0, covar=0;
	      for(int j=0;j<pass;j++){
	      // this part takes sum; should move to GTR/LTR loop later
	      xsum+=result_matrix(j,0);
	      ysum+=result_matrix(j,1);
	      }
	      double xmean=xsum/P;
	      double ymean=ysum/P;
	      for(int j=0;j<pass;j++){
	      xvar+=pow((result_matrix(j,0)-xmean),2);
	      yvar+=pow((result_matrix(j,1)-ymean),2);
	      covar+=((result_matrix(j,0)-xmean)*(result_matrix(j,1)-ymean));
	      }
	      xvar = xvar/(P-1);
	      yvar = yvar/(P-1);
	      covar = covar/(P-1);
	    */
	    temp.peak[tempPeakNum].weight = result * d1.peak[i].weight * d2.peak[j].weight;
	    temp.peak[tempPeakNum].peakID = d2.peak[j].peakID;
	    cout << "DEBUG: tempPeakNum = " << tempPeakNum << endl;
	    cout << "DEBUG: mean[0] = " << temp.peak[tempPeakNum].mean[0]<< endl;
                  /*
		    temp.peak[tempPeakNum].mean.conservativeResize(2);
		    temp.peak[tempPeakNum].mean[0]=xmean;
		    temp.peak[tempPeakNum].mean[1]=ymean;
		    temp.peak[tempPeakNum].variance.conservativeResize(2,2);
		    temp.peak[tempPeakNum].variance(0,0) = xvar;
		    temp.peak[tempPeakNum].variance(0,1) = covar;
		    temp.peak[tempPeakNum].variance(1,0) = covar;
		    temp.peak[tempPeakNum].variance(1,1) = yvar;
		    //			temp.peak[i].variance = (result) * temp.peak[i].variance;
		    //			temp.condOp = d2condOp;
		    //			temp.condBound = d2bound;
		    temp.peak[tempPeakNum].freeze = 0;
		    temp.peak[tempPeakNum].mean += d1.peak[i].mean;
		    temp.peak[tempPeakNum].variance += d1.peak[i].variance;
		    cout << "DEBUG: mean[0] = " << temp.peak[tempPeakNum].mean[0]<<endl;
		    cout << "DEBUG: weight = " << temp.peak[tempPeakNum].weight<<endl;
		  */
	  }
	}
	else{//condOP > 100
	  if(d1.peak[i].peakID == d2.peak[j].peakID){
	    cout << "Match!!" << endl;
	    cout << "Convolving...\n";
	    temp.peak[tempPeakNum].peakID = d1.peak[i].peakID;
	    temp.peak[tempPeakNum].freeze = 0;
	    cout << "Weight check: \n";
	    cout << "d1 -> " << d1.peak[i].weight << endl;
	    cout << "d2 -> " << d2.peak[j].weight << endl;
	    //temp.peak[j+i*d2.peak.size()].weight = d2.peak[j].weight * d1.peak[i].weight;
	    temp.peak[tempPeakNum].weight = d2.peak[j].weight;// * d1.peak[i].weight;
	    cout << "temp->" << temp.peak[j+i*d2.peak.size()].weight << endl;
	    temp.peak[tempPeakNum].mean = d1.peak[i].mean+d2.peak[j].mean;
	    temp.peak[tempPeakNum].variance = d1.peak[i].variance+d2.peak[j].variance;
	  }
	  else if(d2.peak[j].peakID==256){
	    cout << "RHS peak NULL!!" << endl;
	    cout << "Convolving...\n";
	    temp.peak[tempPeakNum].peakID = d1.peak[i].peakID;
	    temp.peak[tempPeakNum].freeze = 0;
	    cout << "Weight check: \n";
	    cout << "d1 -> " << d1.peak[i].weight << endl;
	    cout << "d2 -> " << d2.peak[j].weight << endl;
	    //temp.peak[j+i*d2.peak.size()].weight = d2.peak[j].weight * d1.peak[i].weight;
	    temp.peak[tempPeakNum].weight = d2.peak[j].weight;// * d1.peak[i].weight;
	    cout << "temp->" << temp.peak[j+i*d2.peak.size()].weight << endl;
	    temp.peak[tempPeakNum].mean = d1.peak[i].mean+d2.peak[j].mean;
	    temp.peak[tempPeakNum].variance = d1.peak[i].variance+d2.peak[j].variance;
	  }
	  else if(d1.peak[i].peakID==256){ // dml added fro zeropeak issues
	    cout << "LHS peak NULL!!" << endl;
	    cout << "Convolving...\n";
	    temp.peak[tempPeakNum].peakID = d2.peak[j].peakID;
	    temp.peak[tempPeakNum].freeze = 0;
	    cout << "Weight check: \n";
	    cout << "d1 -> " << d1.peak[i].weight << endl;
	    cout << "d2 -> " << d2.peak[j].weight << endl;
	    //temp.peak[j+i*d2.peak.size()].weight = d2.peak[j].weight * d1.peak[i].weight;
	    temp.peak[tempPeakNum].weight = d2.peak[j].weight;// * d1.peak[i].weight;
	    cout << "temp->" << temp.peak[j+i*d2.peak.size()].weight << endl;
	    temp.peak[tempPeakNum].mean = d1.peak[i].mean+d2.peak[j].mean; //convolve
	    temp.peak[tempPeakNum].variance = d1.peak[i].variance+d2.peak[j].variance;
	  }
	  else{
	    temp.peak[tempPeakNum].weight = 0;
	  } 
	}
      }
      //else if(d1.peak[i].freeze==1){
      //temp.peak[i].freeze = 2;
      //temp.peak[i].weight = d2.peak[j].weight * d1.peak[i].weight;
      //temp.peak[i].mean = d1.peak[i].mean+d2.peak[j].mean;
      //temp.peak[i].variance = d1.peak[i].variance+d2.peak[j].variance;
      //}
      else{
	cout << "ERROR in MoG*Cond!! Case not found.\n";
	d1.print();
	d2.print();
	exit(1);
      }
    }
  }
  temp.check();
  temp.print();
  MoG *pNew = new MoG();
  *pNew = temp;
  //	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
  return pNew;
}
/* for condition distribution*/
double condition(int code, norm_dist dist, Vector vect)
{
	double result=0;

	return result;
}
MoG *convolveVM(Vector v2, MoG d1)
{
	string result_name = "("+v2.name + "*" + d1.distribution_name+")";
	MoG temp(result_name, d1.peak.size());
/*
	for (int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
		if (temp.peak[n].freeze!=2){
			for(int i=0;i<v2.point[0].axis.size();i++){
				for(int j=0;j<temp.peak[n].axis.size();j++){
					if(temp.peak[n].axis[j]==v2.point[0].axis[i]){
						temp.peak[n].mean[i] += v2.point[0].value[j];
					}
				}
			}
		}
	}
*/
	for (int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
		for(int i=0;i<d1.peak[n].mean.size();i++)
			temp.peak[n].mean[i] += v2.point[0].value[i];
	}
	temp.check();
// create a new object to pass back as the result
	MoG *pNew = new MoG();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
vector<norm_dist*> _G_dynamic_memory_list; // for garbage collection
/* routine to take two distributions objects and convolve them returning a third
   This routine is called from the interface library which does the casting 
   It returns a pointer to a new distrib object. Since these objects are dynamically allocated
   the pointer to it is added to the _G_dynamic_memory_list for later garbage colection*/


//why does this exist? -dagan
MoG *deconvolveMV(Vector v2, MoG d1)
{
  cout << "-deconvolveMV" << endl;
	string result_name = "("+d1.distribution_name+"-" +v2.name +  ")";
	MoG temp(result_name, d1.peak.size());
	
	for (int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
//		temp.peak[n].freeze = 1;  // pre-freeze
		for(int i=0;i<v2.point[0].axis.size();i++){
			for(int j=0;j<temp.peak[0].axis.size();j++){
				if(temp.peak[n].axis[j]==v2.point[0].axis[i]){
					temp.peak[n].mean[i] = temp.peak[n].mean[i]-v2.point[0].value[j];
				}
			}
		}
	}
//	temp.check();
	temp.print();
// create a new object to pass back as the result
	MoG *pNew = new MoG();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
MoG *deconvolveVM(Vector v1, MoG d1)
{
  cout << "-deconvolveVM" << endl;
  string result_name = "("+d1.distribution_name+"-" +v1.name +  ")";
	MoG temp(result_name, d1.peak.size());
	
	for (int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
//		temp.peak[n].freeze = 1;  // pre-freeze
		for(int i=0;i<v1.point[0].axis.size();i++){
			for(int j=0;j<temp.peak[0].axis.size();j++){
				if(temp.peak[n].axis[j]==v1.point[0].axis[i]){
					temp.peak[n].mean[i] = v1.point[0].value[j]-temp.peak[n].mean[i];
				}
			}
		}
	}
//	temp.check();
	temp.print();
// create a new object to pass back as the result
	MoG *pNew = new MoG();
/*	string result_name = "("+v1.name + "-" + d1.distribution_name+")";
	Vector temp(result_name);
	temp.name = result_name;
	temp.point[0] = v1.point[0];
	for(int i=0;i<temp.point[0].axis.size();i++){
		temp.point[0].value[i] -= d1.peak[0].mean[i];
	}	
	Vector *pNew = new Vector();*/
	*pNew = temp;
	return pNew;
}

MoG *deconvolveVM_deprecated(Vector v2, MoG d1)
{
	string result_name = "("+v2.name + "-" + d1.distribution_name+")";
	MoG temp(result_name, d1.peak.size());
	for (int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
		if (temp.peak[n].freeze!=2){
			for(int i=0;i<v2.point[0].axis.size();i++){
				for(int j=0;j<temp.peak[0].axis.size();j++){
					if(temp.peak[n].axis[j]==v2.point[0].axis[i]){
						temp.peak[n].mean[i] = v2.point[0].value[j]-temp.peak[n].mean[i];
					}
				}
			}
		}
	}
//	temp.check();
	temp.print();
// create a new object to pass back as the result
	MoG *pNew = new MoG();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

distribution *convolve_single_peak(const distribution &peak1, const distribution &peak2){
  //factor out the peak / peak convolution that happens in MoGs
  distribution temp;
  temp = peak1;
  temp.mean = peak1.mean+peak2.mean;
  temp.variance = peak1.variance+peak2.variance;
  
  //just following structure of most functions here .. but don't think this is necessary...just delete temp later and return res
  distribution *res = new distribution;
  *res = temp;
  return res;
}

MoG * convolveMM (MoG d1, MoG d2)
{
  int num_peak = d1.peak.size() * d2.peak.size();
  string result_name = "convolve(" + d1.distribution_name + "*" + d2.distribution_name + ")";
  MoG temp(result_name, num_peak);
  cout << "ConvolveMM:\n";
  d1.print();
  d2.print();

  cout << "convolveMM function call" << endl;

  double w_normfactor=0;
  // initializing temp and accumulating a normalization factor
  for (int i=0;i<d1.peak.size();i++){
    for (int j=0;j<d2.peak.size();j++){      
      int n = j+i*d2.peak.size();      
      //temp.peak[n] = d1.peak[i];
      if ( (  d1.peak[i].peakID == d2.peak[j].peakID ||
	      d1.peak[i].peakID == 256 ||
	      d2.peak[j].peakID == 256)
	   && d1.peak[i].freeze == 0 && d2.peak[j].freeze == 0)
	w_normfactor += d1.peak[i].weight * d2.peak[j].weight;   
    }
  }


  vector<distribution>::iterator temp_it = temp.peak.begin();
  for (vector<distribution>::iterator lhs_it = d1.peak.begin(); lhs_it != d1.peak.end(); lhs_it++){
    for (vector<distribution>::iterator rhs_it = d2.peak.begin(); rhs_it != d2.peak.end(); rhs_it++){ 
      cout << "d1[" <<lhs_it-d1.peak.begin()<<"] * d2[ " <<rhs_it-d2.peak.begin()
	   <<" ] :: with colors " <<
        lhs_it->peakID << " and " << rhs_it->peakID << endl;
      if(lhs_it->freeze == 0 && rhs_it->freeze == 0){
	if(lhs_it->peakID == rhs_it->peakID){
	  *temp_it =  ( *convolve_single_peak(*lhs_it, *rhs_it)) ;
	  temp_it->peakID = lhs_it->peakID;
	  //temp_it->weight = (lhs_it->weight * rhs_it->weight) / w_normfactor;
	  temp_it->weight = rhs_it->weight;
	  temp_it->freeze = 0;
	  //temp.peak.push_back ( *convolve_single_peak(*lhs_it, *rhs_it)) ;
	  if (temp_it != temp.peak.end()) temp_it++;
	}
	else if(lhs_it->peakID >= 50){
	  *temp_it =  ( *convolve_single_peak(*lhs_it, *rhs_it)) ;
	  temp_it->peakID = rhs_it->peakID;
	  //temp_it->weight = (lhs_it->weight * rhs_it->weight) / w_normfactor;
	  temp_it->weight = rhs_it->weight;
	  temp_it->freeze = 0;
	  //temp.peak.push_back ( *convolve_single_peak(*lhs_it, *rhs_it)) ;
	  if (temp_it != temp.peak.end()) temp_it++;

	}
	else if(rhs_it->peakID >= 50){
	  *temp_it =  ( *convolve_single_peak(*lhs_it, *rhs_it)) ;
	  temp_it->peakID = lhs_it->peakID;
	  //temp_it->weight = (lhs_it->weight * rhs_it->weight) / w_normfactor;
	  temp_it->weight = lhs_it->weight;
	  temp_it->freeze = 0;
	  //temp.peak.push_back ( *convolve_single_peak(*lhs_it, *rhs_it)) ;
	  if (temp_it != temp.peak.end()) temp_it++;
	}
	else {  temp_it->weight = 0; temp_it++;  } //this case should catch nothing

      }
      else if(lhs_it->freeze == 1 || rhs_it->freeze == 1){
	cout << "frozen copy code from below" << endl;
	//set
	//inc tempit
      }
      else if(lhs_it->freeze == 2){
	//just copy lhs
	//inctemp
      }
      else if(rhs_it->freeze == 2){
	//just copy rhs
	//inc temp
      }
      
      //*temp_it = *lhs_it; //give it some arbitrary peak value...does not matter
      //temp_it.weight = 0;

    }
   }
   temp.check();
  //temp.color();
  MoG *pNew = new MoG();
  *pNew = temp;
  return pNew;	
  /****************************************************************************************************/

}


MoG * convolveMM_old (MoG d1, MoG d2)
{
  int num_peak = d1.peak.size() * d2.peak.size();
  string result_name = "(" + d1.distribution_name + "*" + d2.distribution_name + ")";
  MoG temp(result_name, num_peak);
  cout << "ConvolveMM:\n";
  d1.print();
  d2.print();
  
  /*
  //unordered_map<int,double> normalizer; //key <- peakID, value <- sum of all weights for peaks of peakID
  //dagan..
  map<int,double> normalizer; //key <- peakID, value <- sum of all weights for peaks of peakID
  for (int p=0; p<d2.peak.size();p++)
    if (d2.peak[p].freeze == 0)
      //can we assume that map operator[] will default construct val if not exist? for double == 0.0 ??
      normalizer[d2.peak[p].peakID] += d2.peak[p].weight;
  */
  
  double w_normfactor=0;
  // initializing temp and accumulating a normalization factor
  for (int i=0;i<d1.peak.size();i++){
    for (int j=0;j<d2.peak.size();j++){      
      int n = j+i*d2.peak.size();      
      temp.peak[n] = d1.peak[i];
      if ( (  d1.peak[i].peakID == d2.peak[j].peakID ||
	      d1.peak[i].peakID == 256 ||
	      d2.peak[j].peakID == 256)
	   && d1.peak[i].freeze == 0 && d2.peak[j].freeze == 0)
	
	w_normfactor += d1.peak[i].weight * d2.peak[j].weight;
      
    }
  }
  for(int i=0;i<d1.peak.size();i++){
    for(int j=0;j<d2.peak.size();j++){
      int n = j+i*d2.peak.size();
      cout << "d1.peak[" << i << "](peakID=" << d1.peak[i].peakID << ") "
	   << " vs d2.peak[" << j << "](peakID=" << d2.peak[j].peakID << ")...\n";
      if(d1.peak[i].freeze == 0 && d2.peak[j].freeze == 0){
	if(d1.peak[i].peakID == d2.peak[j].peakID){
	  temp.peak[n] = *convolve_single_peak(d1.peak[i], d2.peak[j]);
	  temp.peak[n].weight = (d1.peak[i].weight * d2.peak[j].weight) / w_normfactor;
	  /*cout << "Match!!" << endl;
	  cout << "Convolving...\n";
	  temp.peak[n].peakID = d1.peak[i].peakID;
	  temp.peak[n].freeze = 0;
	  //cout << "Weight check: \n";
	  //cout << "d1 -> " << d1.peak[i].weight << endl;
	  //cout << "d2 -> " << d2.peak[j].weight << endl;
	  //temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
	  //temp.peak[n].weight = (d2.peak[j].weight / normalizer[d2.peak[j].peakID]) * d1.peak[i].weight;
	  temp.peak[n].weight = (d1.peak[i].weight * d2.peak[j].weight) / w_normfactor;
	  cout << "temp->" << temp.peak[n].weight << endl;
	  temp.peak[n].mean = d1.peak[i].mean+d2.peak[j].mean;
	  temp.peak[n].variance = d1.peak[i].variance+d2.peak[j].variance;*/
	}
	else if(d2.peak[j].peakID>=50){ // added by James to avoid zero peak issues
	  cout << "RHS peak uncolored" << endl;
	  cout << "Convolving...\n";
	  temp.peak[n].peakID = d1.peak[i].peakID;
	  temp.peak[n].freeze = 0;
	  cout << "Weight check: \n";
	  cout << "d1 -> " << d1.peak[i].weight << endl;
	  cout << "d2 -> " << d2.peak[j].weight << endl;
	  //temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
	  //temp.peak[n].weight = d2.peak[j].weight;// * d1.peak[i].weight;
	  
	  //temp.peak[n].weight = (w_normfactor == 0) ? d1.peak[i].weight * d2.peak[j].weight : (d1.peak[i].weight * d2.peak[j].weight / w_normfactor); //dagan
	  assert(w_normfactor != 0);
	  temp.peak[n].weight = (d1.peak[i].weight * d2.peak[j].weight) / w_normfactor;
	  cout << "temp->" << temp.peak[n].weight << endl;
	  temp.peak[n].mean = d1.peak[i].mean+d2.peak[j].mean;
	  temp.peak[n].variance = d1.peak[i].variance+d2.peak[j].variance;
	}
	else if(d1.peak[i].peakID>=50){ // dml added 12/17/13 to avoid zeropeak izzues
	  cout << "LHS peak uncolored" << endl;
	  cout << "Convolving...(DANGER this code segment untested!! dml 2/18/14)\n";
          // commented OUT because the indices below are/mightbe wrong
	  temp.peak[n].peakID = d2.peak[j].peakID;
	  temp.peak[n].freeze = 0;
	  cout << "Weight check: \n";
	  cout << "d1 -> " << d1.peak[i].weight << endl;
	  cout << "d2 -> " << d2.peak[j].weight << endl;
	  //temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
	  //temp.peak[n].weight = d2.peak[j].weight;// * d1.peak[i].weight;
	  
	  //assert(w_normfactor != 0); //dagan, testing
	  temp.peak[n].weight = (d1.peak[i].weight * d2.peak[j].weight) / w_normfactor;
	  cout << "temp->" << temp.peak[n].weight << endl;
	  temp.peak[n].mean = d1.peak[i].mean+d2.peak[j].mean;
	  temp.peak[n].variance = d1.peak[i].variance+d2.peak[j].variance;
	}
	else{
	  temp.peak[n].weight = 0;
	}
      }
      else if(d1.peak[i].freeze ==1 || d2.peak[j].freeze==1){
	cout << "Seeing pre-freezer, peak freeze.\n";
	temp.peak[n].freeze = 2;
	temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
	temp.peak[n].mean = d1.peak[i].mean+d2.peak[j].mean;
	temp.peak[n].variance = d1.peak[i].variance+d2.peak[j].variance;
      }
      else if (d2.peak[j].freeze == 2){
	cout << "Peak frozen.\n";
	temp.peak[n] = d2.peak[j];
        //				temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
      }
      else if (d1.peak[i].freeze == 2){
	cout << "Peak frozen.\n";
	temp.peak[n] = d1.peak[i];
        //				temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
      }
      else{
	cout << "ERROR! Convolve case not found.\n";
	d1.print();
	d2.print();
	exit(1);
      }
    }
  }
  temp.check();
  //temp.color();
  MoG *pNew = new MoG();
  *pNew = temp;
  return pNew;	
}



// dml 3/3/14
// result = d1-d2, and weights are those of d2
// respects colors and uncolored distributions
// 

MoG * deconvolveMM (MoG d1, MoG d2)
{
  //this multiplication is the maximum size ... 
  //2peak - 2peak where 1st peak of each is color 0 and 2nd peak of each is color 1 should produce 2 peaks -dgn
  int num_peak = d1.peak.size() * d2.peak.size();
  string result_name = "(" + d1.distribution_name + "-" + d2.distribution_name + ")";
  MoG temp(result_name, num_peak);
  cout << "deconvolveMM:\n";
  d1.print();
  d2.print();
  // initializing temp
    double w_normfactor=0;
  // initializing temp and accumulating a normalization factor
  for (int i=0;i<d1.peak.size();i++){
    for (int j=0;j<d2.peak.size();j++){      
      int n = j+i*d2.peak.size();      
      temp.peak[n] = d1.peak[i];
      if ( (  d1.peak[i].peakID == d2.peak[j].peakID ||
	      d1.peak[i].peakID == 256 ||
	      d2.peak[j].peakID == 256)
	   && d1.peak[i].freeze == 0 && d2.peak[j].freeze == 0)	
	w_normfactor += d1.peak[i].weight * d2.peak[j].weight;
      
    }
  }

  for(int i=0;i<d1.peak.size();i++){
    for(int j=0;j<d2.peak.size();j++){
      cout << "d1.peak[" << i << "](peakID=" << d1.peak[i].peakID << ") "
	   << " vs d2.peak[" << j << "](peakID=" << d2.peak[j].peakID << ")...\n";
      int n = j+i*d2.peak.size(); // index of temp peak
      if(d1.peak[i].freeze == 0 && d2.peak[j].freeze == 0){
	if(d1.peak[i].peakID == d2.peak[j].peakID){
	  cout << "Match!!" << endl;
	  cout << "DeConvolving...\n";
	  temp.peak[n].peakID = d1.peak[i].peakID;
	  temp.peak[n].freeze = 0;
	  temp.peak[n].weight = d2.peak[j].weight;// * d1.peak[i].weight;
	  //temp.peak[n].weight = (d1.peak[i].weight * d2.peak[j].weight) / w_normfactor; //dagan
	  cout << "temp["<<n<<"]-> weight=" << temp.peak[n].weight << endl;
	  temp.peak[n].mean = d1.peak[i].mean-d2.peak[j].mean;
	  temp.peak[n].variance = d1.peak[i].variance+d2.peak[j].variance;
	}
	else if(d1.peak[i].peakID >= 256){
	  cout << "Match!!" << endl;
	  cout << "DeConvolving w/uncolored d1\n";
	  temp.peak[n].peakID = d2.peak[j].peakID;
	  temp.peak[n].freeze = 0;
	  //temp.peak[n].weight = d1.peak[i].weight * d2.peak[j].weight;
	  temp.peak[n].weight = d2.peak[j].weight;
	  //temp.peak[n].weight = (d1.peak[i].weight * d2.peak[j].weight) / w_normfactor;
	  cout << "temp["<<n<<"]-> weight=" << temp.peak[n].weight << endl;
	  temp.peak[n].mean = d1.peak[i].mean-d2.peak[j].mean;
	  temp.peak[n].variance = d1.peak[i].variance+d2.peak[j].variance;
	}
	else if(d2.peak[j].peakID >= 256){
	  cout << "Match!!" << endl;
	  cout << "DeConvolving w/uncolered d2\n";
	  temp.peak[n].peakID = d1.peak[i].peakID;
	  temp.peak[n].freeze = 0;
	  //temp.peak[n].weight = d1.peak[i].weight * d2.peak[j].weight;
	  temp.peak[n].weight = d1.peak[i].weight;
	  //temp.peak[n].weight = (d1.peak[i].weight * d2.peak[j].weight) / w_normfactor;
	  cout << "temp["<<n<<"]-> weight=" << temp.peak[n].weight << endl;
	  temp.peak[n].mean = d1.peak[i].mean-d2.peak[j].mean;
	  temp.peak[n].variance = d1.peak[i].variance+d2.peak[j].variance;
	}
	else temp.peak[n].weight = 0;	//what case does this cover? non-matching colors?
      }// end of both not frozen

      else if(d1.peak[i].freeze ==1 || d2.peak[j].freeze==1){
	cout << "Seeing pre-freezer, peak freeze.\n";
	temp.peak[n].freeze = 2;
	temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
	temp.peak[n].mean = d1.peak[i].mean-d2.peak[j].mean;
	temp.peak[n].variance = d1.peak[i].variance-d2.peak[j].variance;
      }
      else if (d2.peak[j].freeze == 2){
	cout << "Peak frozen.\n";
	temp.peak[n] = d2.peak[j];
        //  temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
      }
      else if (d1.peak[i].freeze == 2){
	cout << "Peak frozen.\n";
	temp.peak[n] = d1.peak[i];
        //  temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
      }
      else{
	cout << "ERROR! Convolve case not found.\n";
	d1.print();
	d2.print();
	exit(1);
      }
    }
  }
  temp.check();
  //temp.color();
  MoG *pNew = new MoG();
  *pNew = temp;
  return pNew;	
}
//
// Modified Feb 2014 dml to take two colored mogs
// Each peak i in d1 is rotated by peak i or peak 0 in d2
// DOES NOT RESPECT COLORS very well. 
// color of result given by color of rotation entry 

MoG *rotateMM(MoG d1, MoG d2)
{
  bool match_flag = false, multiplePeakFlag=false;
  int match_counter =0;
  string result_name = "(" + d1.distribution_name + "^" + d2.distribution_name  + ")";
  MoG temp(result_name, d2.peak.size());

  cout<<result_name<<endl;
  cout << "rotateMM: d1 #p="<<d1.peak.size()<<" d2 #p="<<d2.peak.size()<<endl;

  if (d1.peak.size()>1) 
    if (d1.peak.size()==d2.peak.size()) multiplePeakFlag=true;
    else
      cout << "TSS: RotateMM: ERROR - d2 has more than 1 peak but not the same number as d1!!\n";
  

  if (multiplePeakFlag)
    cout<<"Rotate: operating with corresponding multiple peaks in d1 and d2\n";
  else
    cout<<"Rotate: all peaks in d1 rotated by peak[0] of d2\n";

  int i1=0,i2=0;
  for (i1=0;i1<temp.peak.size();i1++){
    cout<<"Rotate d1["<<i1<<"] by d2["<<i2<<"]\n";
    temp.peak[i1] = d1.peak[i2];
    cout << "octopus : after assignment, temp.peak[" << i1 << "].weight = " << temp.peak[i1].weight << endl;
    temp.peak[i1].peakID=d2.peak[i1].peakID; // color comes from rotation
    /* NEED TO REWORK THIS PART	
       TO DO 1)ROTATE BACK TO 0 ANGLE 2)ROTATE TO d2*/
    if (d2.peak.size()>=0 && d2.peak[i1].axis.size()==2 && temp.peak[i1].variance(0,1)==0){
      int vect_match_axis[2];
      int dist_match_axis[2];
      int match_counter =0;
      for(int i=0;i<d2.peak[i1].axis.size();i++){
	for(int j=0;j<temp.peak[i1].axis.size();j++){
	  if(temp.peak[i1].axis[j]==d2.peak[i1].axis[i]){
	    //						temp.peak[n].mean[i] += d2.peak[0].mean[j];
	    vect_match_axis[match_counter]=i;
	    dist_match_axis[match_counter]=j;
	    match_counter++;
	  }
	}// for j
      }// for i
      double rotation_angle=atan2(d2.peak[i1].mean(1), d2.peak[i1].mean(0)); // this is in radians
      cout << "Rotational angle going into rotateMM: "<< rotation_angle*57.29578 <<endl;
      double sin_v = sin(rotation_angle);
      double cos_v = cos(rotation_angle);
      Matrix2d rotation_matrix;
      /* rotation_matrix << cos_v, sin_v, -sin_v, cos_v; forgot which one rotation matrix is correct?*/
      cout << "Pre Rotated mean values: " << temp.peak[i1].mean << endl;
      cout << "cos_v, sin_v: " << cos_v << ","<< sin_v <<endl;
      rotation_matrix << cos_v, -sin_v, sin_v, cos_v;
      double tmp1 = cos_v * temp.peak[i1].mean[0] - sin_v * temp.peak[i1].mean[1];
      double tmp2 = sin_v * temp.peak[i1].mean[0] + cos_v * temp.peak[i1].mean[1];
      temp.peak[i1].mean[0] = tmp1;
      temp.peak[i1].mean[1] = tmp2;
      cout << "Rotated mean values: " << temp.peak[i1].mean << endl;
      Matrix2d covariance_matrix;
      covariance_matrix <<  temp.peak[i1].variance(dist_match_axis[0],dist_match_axis[0])
	, temp.peak[i1].variance(dist_match_axis[0],dist_match_axis[1])
	, temp.peak[i1].variance(dist_match_axis[1],dist_match_axis[0])
	, temp.peak[i1].variance(dist_match_axis[1],dist_match_axis[1]);
      covariance_matrix = rotation_matrix * covariance_matrix * rotation_matrix.inverse();
      cout << covariance_matrix << endl;	
      temp.peak[i1].variance(dist_match_axis[0],dist_match_axis[0]) = covariance_matrix(0,0);
      temp.peak[i1].variance(dist_match_axis[0],dist_match_axis[1]) = covariance_matrix(0,1);
      temp.peak[i1].variance(dist_match_axis[1],dist_match_axis[0]) = covariance_matrix(1,0);
      temp.peak[i1].variance(dist_match_axis[1],dist_match_axis[1]) = covariance_matrix(1,1);
    }// if d2.peak.size>0 etc
    else
      cout << "TSS: RotateMM; ERROR!! Please check vector or distribution orientation.\n";

    if (multiplePeakFlag) i2++;
  }// for i1
  temp.check();
  MoG *pNew = new MoG();
  *pNew = temp;
  return pNew;
}


MoG *rotateVM(Vector v2, MoG d1)
{
	bool match_flag = false;
	int match_counter =0;
	string result_name = "(" + d1.distribution_name + "^" + v2.name  + ")";
	MoG temp(result_name, d1.peak.size());

	for (int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
	/* NEED TO REWORK THIS PART	
		TO DO 1)ROTATE BACK TO 0 ANGLE 2)ROTATE TO V2*/
		if (v2.point[0].axis.size()==2 && temp.peak[n].variance(0,1)==0){
			int vect_match_axis[2];
			int dist_match_axis[2];
			int match_counter =0;
			for(int i=0;i<v2.point[0].axis.size();i++){
				for(int j=0;j<temp.peak[n].axis.size();j++){
					if(temp.peak[n].axis[j]==v2.point[0].axis[i]){
//						temp.peak[n].mean[i] += v2.point[0].value[j];
						vect_match_axis[match_counter]=i;
						dist_match_axis[match_counter]=j;
						match_counter++;
					}
				}
			}
			if(match_counter!=2) exit(1);
			double rotation_angle=atan2(v2.point[0].value[1], v2.point[0].value[0]); // this is in radians
			cout << "Rotational angle going into rotateVM: "<< rotation_angle*57.29578 <<endl;
			double sin_v = sin(rotation_angle);
			double cos_v = cos(rotation_angle);
			Matrix2d rotation_matrix;
			/* rotation_matrix << cos_v, sin_v, -sin_v, cos_v; forgot which one rotation matrix is correct?*/
			rotation_matrix << cos_v, -sin_v, sin_v, cos_v;
			temp.peak[n].mean[0] = cos_v * temp.peak[n].mean[0] - sin_v * temp.peak[n].mean[1];
			temp.peak[n].mean[1] = sin_v * temp.peak[n].mean[0] + cos_v * temp.peak[n].mean[1];
			cout << "Rotated mean values: " << temp.peak[n].mean << endl;
			Matrix2d covariance_matrix;
			covariance_matrix <<  temp.peak[n].variance(dist_match_axis[0],dist_match_axis[0])
								, temp.peak[n].variance(dist_match_axis[0],dist_match_axis[1])
								, temp.peak[n].variance(dist_match_axis[1],dist_match_axis[0])
								, temp.peak[n].variance(dist_match_axis[1],dist_match_axis[1]);
/*			covariance_matrix <<  temp.peak[n].variance(0,0)
								, temp.peak[n].variance(0,1)
								, temp.peak[n].variance(1,0)
								, temp.peak[n].variance(1,1);
*/	//		covariance_matrix = rotation_matrix.inverse() * covariance_matrix * rotation_matrix;
			covariance_matrix = rotation_matrix * covariance_matrix * rotation_matrix.inverse();
	
	//	cout << covariance_matrix << endl;
/*			double scale = sqrt(pow(v2.point[0].value[vect_match_axis[0]],2)+pow(v2.point[0].value[vect_match_axis[1]],2));
			for(int i=0;i<2;i++){
				for(int j=0;j<2;j++){
					covariance_matrix(i,j) = scale * scale * covariance_matrix(i,j);
				}
			}
			*/
			cout << covariance_matrix << endl;	
			temp.peak[n].variance(0,0) = covariance_matrix(0,0);
			temp.peak[n].variance(0,1) = covariance_matrix(0,1);
			temp.peak[n].variance(1,0) = covariance_matrix(1,0);
			temp.peak[n].variance(1,1) = covariance_matrix(1,1);
		}
		else
			cout << "ERROR!! Please check vector or distribution orientation.\n";
	}
	MoG *pNew = new MoG();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
MoG * disjunctMM (MoG d1, MoG d2)
{
	int num_peak = d1.peak.size() + d2.peak.size();
	string result_name = "(" + d1.distribution_name + "+" + d2.distribution_name + ")";
	MoG temp(result_name, num_peak);
	for(int i=0; i<d1.peak.size();i++){
		temp.peak[i]=d1.peak[i];
	}
	for(int i=0; i<d2.peak.size();i++){
		temp.peak[i+d1.peak.size()]=d2.peak[i];
	}
	temp.check();
//	temp.balance();
	temp.print();
	checkpoint();
	MoG *pNew = new MoG();
	*pNew = temp;
	return pNew;
}
MoG * disjunctZZ (MoG d1, MoG d2)
{
	int num_peak = d1.peak.size() + d2.peak.size();
	string result_name = "(" + d1.distribution_name + "+" + d2.distribution_name + ")";
	MoG temp(result_name, num_peak);
	for(int i=0; i<d1.peak.size();i++){
		temp.peak[i]=d1.peak[i];
	}
	for(int i=0; i<d2.peak.size();i++){
		temp.peak[i+d1.peak.size()]=d2.peak[i];
	}
	temp.check();
	temp.print();
	MoG *pNew = new MoG();
	*pNew = temp;
	return pNew;
}

MoG *scaleM(double p2, MoG d1)
{
	stringstream s;
	s << p2;
	string result_name = "scaleM("+s.str() + "*" + d1.distribution_name+")";
	MoG temp(result_name, d1.peak.size());
	for(int n=0;n<d1.peak.size();n++){
	  temp.peak[n] = d1.peak[n];
	  if (temp.peak[n].freeze==0){
	    for(int i=0;i<temp.peak[n].axis.size();i++){
	      temp.peak[n].mean[i] = d1.peak[n].mean[i]*p2;
	      // dml SEP 2013 enabled r * m includes variance scaling
	      for(int j=0;j<temp.peak[n].axis.size();j++){
		temp.peak[n].variance(i,j) = d1.peak[n].variance(i,j)*pow(p2,2);		       
	      }
	    }
	  }
	  else if(temp.peak[n].freeze==1){
	    for(int i=0;i<temp.peak[n].axis.size();i++){
	      temp.peak[n].mean[i] = d1.peak[n].mean[i]*p2;
	    }
	    temp.peak[n].freeze = 2;
	  }
	  else if(temp.peak[n].freeze==2){
	  }
	}
	temp.check();
	//	checkpoint();
	MoG *pNew = new MoG();
	*pNew = temp;
	//_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

MoG *scaleZ(double p2, MoG d1)
{
	d1.check();
	stringstream s;
	s << p2;
	string result_name = "scaleZ("+s.str() + "*" + d1.distribution_name+")";
	MoG temp(result_name, d1.peak.size());
	for(int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
		if (temp.peak[n].freeze==2){
			// freeze; do nothing
		}
		else if (temp.peak[n].freeze==1){
			temp.peak[n].freeze=2;
			// from pre-freeze to freeze
		}
		else if (temp.peak[n].freeze==0){
			temp.peak[n] = d1.peak[n];
			for(int i=0;i<temp.peak[n].axis.size();i++){
				temp.peak[n].mean[i] = d1.peak[n].mean[i]*p2;
			}
/*			int d1condOp = d1.condOp;
			Vector * d1bound = d1.condBound;
			double Wall_1_x = d1bound->point[0].value[0];
			double Wall_1_y = d1bound->point[0].value[1];
			for(int i=0;i<d1.peak.size();i++){
				int pass=0;
				int n = 5000;
				double value_Wall_1;
				MatrixXd tempPts = d1.mvnrng(i,n);
				MatrixXd result_matrix;
				// case 1: GTR wall_1
				if(d1condOp==1){
					for(int j=0;j<n;j++){
						value_Wall_1=Wall_1_x*tempPts(j,0)+Wall_1_y*tempPts(j,1)-(pow(Wall_1_x,2)+pow(Wall_1_y,2));
						if(value_Wall_1>0){
							pass++;
							result_matrix.conservativeResize(pass,2);
							result_matrix(pass-1,0)=tempPts(j,0);
							result_matrix(pass-1,1)=tempPts(j,1);
						}
					}
				}					
				// case 2: LTR wall_1
				else if(d1condOp==2){
					for(int j=0;j<n;j++){
						value_Wall_1=Wall_1_x*tempPts(j,0)+Wall_1_y*tempPts(j,1)-(pow(Wall_1_x,2)+pow(Wall_1_y,2));
						if(value_Wall_1<0){
							pass++;
							result_matrix.conservativeResize(pass,2);
							result_matrix(pass-1,0)=tempPts(j,0);
							result_matrix(pass-1,1)=tempPts(j,1);
						}
					}
				}
				double P=pass+00000001;
				double N=n+0.01;
				double result = P/N;
				if (result <= 0.005)
					result = 0;
				if (result >= 0.995)
					result = 1;	
				double xsum=0,ysum=0, xvar=0, yvar=0, covar=0;
				for(int j=0;j<pass;j++){
				    xsum+=result_matrix(j,0);
				    ysum+=result_matrix(j,1);
				}
				double xmean=xsum/P;
				double ymean=ysum/P;
				temp.peak[i].weight = result * d1.peak[i].weight;
				temp.peak[i].mean.conservativeResize(2);
				temp.peak[i].mean[0]=xmean;
				temp.peak[i].mean[1]=ymean;
				for(int j=0;j<pass;j++){
					xvar+=pow((result_matrix(j,0)-xmean),2);
					yvar+=pow((result_matrix(j,1)-ymean),2);
					covar+=((result_matrix(j,0)-xmean)*(result_matrix(j,1)-ymean));
				}
				xvar = xvar/(P-1);
				yvar = yvar/(P-1);
				covar = covar/(P-1);
				temp.peak[i].variance.conservativeResize(2,2);
				temp.peak[i].variance(0,0) = xvar;
				temp.peak[i].variance(0,1) = covar;
				temp.peak[i].variance(1,1) = yvar;
				temp.peak[i].variance(1,0) = covar;
//				temp.peak[i].variance = sqrt(result) * temp.peak[i].variance;	

	/*			if((a*mu[0]+b*mu[1]-(a*a+b*b))>0){ 		// if mean ls greater than the bound
					cout << "Separating the peak..."<< result << endl;
					double slope = b/a;
					double offset = (a*mu[0]+b*mu[1]-(a*a+b*b))/(a+b*slope);
					peak[i].mean[0] += (offset );//+ result*offset);
					peak[i].mean[1] += (slope*offset);// + (result*slope*offset));
				}*/
				
//				temp.condOp = d1condOp;
//				temp.condBound = d1bound;
//				}
//				}
			}
			else{
				cout << "ERROR! Operation not found.\n";
				exit(1);
			}
	}
	temp.check();
	temp.print();
	MoG *pNew = new MoG();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
Vector *scaleV(double p2, Vector v1)
{
  cout << "debug - scaleV called on " << v1.name << endl;
  stringstream s;
  s << p2;
  string result_name = "(" + v1.name + "*" + s.str() + ")";
  Vector temp(result_name);
  temp.name = result_name;
  temp.point[0] = v1.point[0];
  for(int i=0;i<temp.point[0].axis.size();i++){
    temp.point[0].value[i] = v1.point[0].value[i]*p2;
  }	
  
  Vector *pNew = new Vector();
  *pNew = temp;
  /* record the pointer for later garbage collection
   * should also do a vector version of garbage collection
   _G_dynamic_memory_list.push_back(pNew); 
  */	return pNew;
}

Vector *scaleVV(Vector v1, Vector v2)
{
	stringstream s;
	string result_name = v1.name + "*" + v2.name;
	Vector temp(result_name);
	temp.name = result_name;
	temp.point[0] = v1.point[0];
	for(int i=0;i<temp.point[0].axis.size();i++){
//		for(int j=0;j<v2.point[0].axis.size();j++){
//			if(temp.point[0].axis[i]==v2.point[0].axis[j])
				temp.point[0].value[i] = v1.point[0].value[i] * v2.point[0].value[i];
//		}
	}
	Vector *pNew = new Vector();
	*pNew = temp; 
// record the pointer for later garbage collection
// should also do a vector version of garbage collection
//	_G_dynamic_memory_list.push_back(pNew); 
	return pNew;
}
Vector *scaleVM(Vector v, MoG d)
{
	stringstream s;
	string result_name = "("+v.name + "*" + d.distribution_name+")";
	Vector temp(result_name);
	temp.name = result_name;
	temp.point[0] = v.point[0];
        double ratio = 0;
        for(int i=0;i<d.peak.size();i++){
                if(d.peak[i].freeze==0){	
                        double result = d.peak[i].weight;
                        ratio += result;
                }
        }
        
	for(int i=0;i<temp.point[0].axis.size();i++){
                temp.point[0].value[i] = v.point[0].value[i] * ratio;
	}

	Vector *pNew = new Vector();
	*pNew = temp;
// record the pointer for later garbage collection
// should also do a vector version of garbage collection
//	_G_dynamic_memory_list.push_back(pNew); 
	return pNew;
}
Vector *scaleVZ(Vector v, MoG d)
{
	stringstream s;
	string result_name = "("+v.name + "*" + d.distribution_name+")";
	Vector temp(result_name);
	temp.name = result_name;
	temp.point[0] = v.point[0];
        double ratio = 0;
        for(int i=0;i<d.peak.size();i++){
                if(d.peak[i].freeze==0){	
                        int dcondOp = d.condOp;
                        Vector * dbound = d.condBound;
                        double Wall_2_x = dbound->point[0].value[0];
                        double Wall_2_y = dbound->point[0].value[1];
                        int pass=0;
                        int n = 4000;
                        double value_Wall_2;
                        MatrixXd tempPts = d.mvnrng(i,n);
                        MatrixXd result_matrix;
                        // case 1: GTR wall_2
                        if(dcondOp==1){
                                for(int j=0;j<n;j++){
                                        value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
                                        if(value_Wall_2>0){
                                                pass++;
                                        }
                                }
                        }
                        // case 2: LTR wall_2
                        else if(dcondOp==2){
                                for(int j=0;j<n;j++){
                                        value_Wall_2=Wall_2_x*tempPts(j,0)+Wall_2_y*tempPts(j,1)-(pow(Wall_2_x,2)+pow(Wall_2_y,2));
                                        if(value_Wall_2<0){
                                                pass++;
                                        }
                                }
                        }
                        else{
                                cout << "Op case not found...please doublecheck pars file.\n";
                                d.print();
                                exit(1);
                        }
                        double P=pass+0.00000001;
                        double N=n+0.01;
                        double result = P/N;
                        if (result <= 0.005)
                                result = 0;
                        if (result >= 0.995)
                                result = 1;	

//                        double result = d.peak[i].weight;
                        ratio += result;
                }
        }
        
	for(int i=0;i<temp.point[0].axis.size();i++){
                temp.point[0].value[i] = v.point[0].value[i] * ratio;
	}

	Vector *pNew = new Vector();
	*pNew = temp;
// record the pointer for later garbage collection
// should also do a vector version of garbage collection
//	_G_dynamic_memory_list.push_back(pNew); 
	return pNew;
}
double dotVV(Vector v1, Vector v2)
{// do dot product
	double pNew=0;
	cout << "Dot product of :\n";
	v1.print(); 
	v2.print();
	if (v1.point[0].value.size()!=v2.point[0].value.size())
		printf("ERROR! Dimension of the two vectors not equal, cannot calculate dot product.\n");
	else{
		for(int i=0;i<v1.point[0].value.size();i++)
			pNew+=v1.point[0].value[i]*v2.point[0].value[i];
	}
	cout << "\n-->" << pNew << endl;
	return pNew;
}

double dotMM(MoG v1, MoG v2)
{// do dot product
	double pNew=0;
	cout << "Dot product of :\n";
	v1.print(); 
	v2.print();
	if (v1.peak[0].axis.size()!=v2.peak[0].axis.size())
		printf("ERROR! Dimension of the two vectors not equal, cannot calculate dot product.\n");
	else{
		for(int i=0;i<v1.peak[0].axis.size();i++)
			pNew+=v1.peak[0].mean[i]*v2.peak[0].mean[i];
	}
	cout << "\n-->" << pNew << endl;
	return pNew;
}



/*
double scaleMM(MoG v1, MoG v2)
{
	double pNew=0;
	cout << "Dot product of :\n";
	v1.print(); 
	v2.print();
	if (v1.peak[0].mean.size()!=v2.peak[0].mean.size())
		printf("ERROR! Dimension of the two MoGs not equal, cannot calculate dot product.\n");
	else{
		for(int i=0;i<v1.peak[0].mean.size();i++)
			pNew+=v1.peak[0].mean[i]*v2.peak[0].mean[i];
	}
	cout << "\n-->" << pNew << endl;
	return pNew;
}
*/
MoG *scaleMM(MoG d1, MoG d2)
{
	string result_name = "("+ d1.distribution_name + "/" + d2.distribution_name+")";
	MoG temp(result_name, d1.peak.size());
	for(int n=0;n<d1.peak.size();n++){
		temp.peak[n] = d1.peak[n];
                //for(int i=0;i<temp.peak[n].axis.size();i++){
		//	temp.peak[n].mean[i] = d1.peak[n].mean[i]*p2;
		//}
                temp.peak[n].mean = d2.peak[0].mean;
	}
	temp.check();
//	checkpoint();
	MoG *pNew = new MoG();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
norm_dist *scaleVD(Vector v2, norm_dist d1)
{/* vector * distribution doesn't make sense!!
	here replaced by vector + distribution */
	string result_name = "("+d1.distribution_name + "*" + v2.name + ")";
	norm_dist temp(result_name);
	temp.peak[0] = d1.peak[0];
	for(int i=0;i<temp.peak[0].axis.size();i++){
		for(int j=0;j<v2.point[0].axis.size();j++){
			if(temp.peak[0].axis[i]==v2.point[0].axis[j])
				temp.peak[0].mean[i] += v2.point[0].value[j];
		}
	}
	cout << "vector * distribution doesn't make sense!!\nHere replaced by vector + distribution\n";
	
	norm_dist *pNew = new norm_dist();
	*pNew = temp;
	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

norm_dist *scaleD(double p2, norm_dist d1)
{
  cout << "debug - scaleD called\n";
  stringstream s;
  s << p2;
  string result_name = d1.distribution_name + "*" + s.str();
  norm_dist temp(result_name);
  temp.peak[0] = d1.peak[0];
  for(int i=0;i<temp.peak[0].axis.size();i++){
    temp.peak[0].mean[i] = d1.peak[0].mean[i]*p2;
    for(int j=0;j<temp.peak[0].axis.size();j++){
      temp.peak[0].variance(i,j) = d1.peak[0].variance(i,j)*pow(p2,2);
    }
  }
  norm_dist *pNew = new norm_dist();
  *pNew = temp;
  _G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
  return pNew;
}   

norm_dist *rotateVD(Vector v2, norm_dist d1)
{
	bool match_flag = false;
	int match_counter =0;
	string result_name = "(" + d1.distribution_name + "^" + v2.name  + ")";
	norm_dist temp(result_name);
	temp.peak[0] = d1.peak[0];
/* NEED TO REWORK THIS PART	
	TO DO 1)ROTATE BACK TO 0 ANGLE 2)ROTATE TO V2*/
	if (v2.point[0].axis.size()==2 && temp.peak[0].variance(0,1)==0){
		int vect_match_axis[2];
		int dist_match_axis[2];
		int match_counter =0;
		for(int i=0;i<v2.point[0].axis.size();i++){
			for(int j=0;j<temp.peak[0].axis.size();j++){
				if(temp.peak[0].axis[j]==v2.point[0].axis[i]){
					temp.peak[0].mean[i] += v2.point[0].value[j];
					vect_match_axis[match_counter]=i;
					dist_match_axis[match_counter]=j;
					match_counter++;
				}
			}
		}
		double rotation_angle=atan(v2.point[0].value[vect_match_axis[0]]/v2.point[0].value[vect_match_axis[1]]); // this is in radians
		double sin_v = sin(rotation_angle);
		double cos_v = cos(rotation_angle);
		Matrix2d rotation_matrix;
		rotation_matrix << cos_v, sin_v, -sin_v, cos_v;
//	cout << rotation_matrix << endl;
		Matrix2d covariance_matrix;
		covariance_matrix <<  temp.peak[0].variance(dist_match_axis[0],dist_match_axis[0])
							, temp.peak[0].variance(dist_match_axis[0],dist_match_axis[1])
							, temp.peak[0].variance(dist_match_axis[1],dist_match_axis[0])
							, temp.peak[0].variance(dist_match_axis[1],dist_match_axis[1]);
		covariance_matrix = rotation_matrix.inverse() * covariance_matrix * rotation_matrix;

//	cout << covariance_matrix << endl;
		double scale = sqrt(pow(v2.point[0].value[vect_match_axis[0]],2)+pow(v2.point[0].value[vect_match_axis[1]],2));
		for(int i=0;i<2;i++){
			for(int j=0;j<2;j++){
				covariance_matrix(i,j) = scale * scale * covariance_matrix(i,j);
			}
		}
	cout << covariance_matrix << endl;	
		temp.peak[0].variance(dist_match_axis[0],dist_match_axis[0]) = covariance_matrix(0,0);
		temp.peak[0].variance(dist_match_axis[0],dist_match_axis[1]) = covariance_matrix(0,1);
		temp.peak[0].variance(dist_match_axis[1],dist_match_axis[0]) = covariance_matrix(1,0);
		temp.peak[0].variance(dist_match_axis[1],dist_match_axis[1]) = covariance_matrix(1,1);
	}
	else
		cout << "ERROR!! Please check vector or distribution orientation.\n";
	norm_dist *pNew = new norm_dist();
	*pNew = temp;
	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
norm_dist *convolveVD(Vector v2, norm_dist d1)
{
	bool match_flag = false;
	int match_counter =0;
	string result_name = "("+v2.name + "+" + d1.distribution_name+")";
	norm_dist temp(result_name);
	temp.peak[0] = d1.peak[0];

	for(int i=0;i<v2.point[0].axis.size();i++){
		for(int j=0;j<temp.peak[0].axis.size();j++){
			if(temp.peak[0].axis[j]==v2.point[0].axis[i]){
				temp.peak[0].mean[i] += v2.point[0].value[j];
				match_flag = true;
			}
		}
		if(match_flag == false){
			temp.peak[0].axis.push_back(v2.point[0].axis[i]);
			temp.peak[0].mean.conservativeResize(temp.peak[0].axis.size());
			temp.peak[0].mean(temp.peak[0].axis.size()-1)=v2.point[0].value[i];
		}
		match_flag = false;
	}
// create a new object to pass back as the result
	norm_dist *pNew = new norm_dist();
	*pNew = temp;
	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

norm_dist *deconvolveDV(Vector v2, norm_dist d1)
{
	bool match_flag = false;
	int match_counter =0;
	string result_name = "("+d1.distribution_name+"-" +v2.name +  ")";
	norm_dist temp(result_name);
	temp.peak[0] = d1.peak[0];

	for(int i=0;i<v2.point[0].axis.size();i++){
		for(int j=0;j<temp.peak[0].axis.size();j++){
			if(temp.peak[0].axis[j]==v2.point[0].axis[i]){
				temp.peak[0].mean[i] = temp.peak[0].mean[i]-v2.point[0].value[j];
				match_flag = true;
			}
		}
		if(match_flag == false){
			temp.peak[0].axis.push_back(v2.point[0].axis[i]);
			temp.peak[0].mean.conservativeResize(temp.peak[0].axis.size());
			temp.peak[0].mean(temp.peak[0].axis.size()-1)=-v2.point[0].value[i];
		}
		match_flag = false;
	}
// create a new object to pass back as the result
	norm_dist *pNew = new norm_dist();
	*pNew = temp;
	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}


norm_dist *deconvolveVD(Vector v2, norm_dist d1)
{
	bool match_flag = false;
	int match_counter =0;
	string result_name = "("+v2.name + "-" + d1.distribution_name+")";
	norm_dist temp(result_name);
	temp.peak[0] = d1.peak[0];

	for(int i=0;i<v2.point[0].axis.size();i++){
		for(int j=0;j<temp.peak[0].axis.size();j++){
			if(temp.peak[0].axis[j]==v2.point[0].axis[i]){
				temp.peak[0].mean[i] = v2.point[0].value[j]-temp.peak[0].mean[i];
				match_flag = true;
			}
		}
		if(match_flag == false){
			temp.peak[0].axis.push_back(v2.point[0].axis[i]);
			temp.peak[0].mean.conservativeResize(temp.peak[0].axis.size());
			temp.peak[0].mean(temp.peak[0].axis.size()-1)=v2.point[0].value[i];
		}
		match_flag = false;
	}
// create a new object to pass back as the result
	norm_dist *pNew = new norm_dist();
	*pNew = temp;
	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

/*
norm_dist *convolveND(double p2, norm_dist d1)
{
	Warning!! This is a very raw calculation, 
	might not be suitable for more than 1 axis!! 
	stringstream s;
	s << p2;
	string result_name = s.str() + "+" + d1.distribution_name;
	norm_dist temp(result_name);
	temp.peak[0] = d1.peak[0];
	for(int i=0;i<temp.peak[0].axis.size();i++)
		temp.peak[0].mean[i] += p2;

	// create a new object to pass back as the result
	norm_dist *pNew = new norm_dist();
	*pNew = temp;
	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
*/
norm_dist *convolveDD(norm_dist d1, norm_dist d2)
{
	string result_name ="("+ d1.distribution_name + "+" + d2.distribution_name+")";
	cout << d1.distribution_name << " + " << d2.distribution_name << endl;
	norm_dist temp;
	temp.distribution_name = result_name;
	temp.peak[0] = d1.peak[0];

	int match_counter =0;
// Examine if the axis match up 
	for(int i=0;i<d2.peak[0].axis.size();i++){
// if match axis is found, then convolve		
		if (binary_search (temp.peak[0].axis.begin(),temp.peak[0].axis.end(), d2.peak[0].axis[i])){
			match_counter++;
		}
	}
	cout << "Finished search;\n";
	if (match_counter==d2.peak[0].axis.size()){
		match_counter=0;
		int d1_match_axis[2];
		int d2_match_axis[2];
		cout << "Found 2 matching axis: convolving...";
		for(int i=0;i<d2.peak[0].axis.size();i++){
			for(int j=0;j<temp.peak[0].axis.size();j++){
				if(d2.peak[0].axis[i]==temp.peak[0].axis[j]){
					cout << "Mean #" << match_counter+1 << "calculating...";
					temp.peak[0].mean[j]=temp.peak[0].mean[j]+d2.peak[0].mean[i];
//					if(temp.peak[0].axis[j]==v2.point[0].axis[i]){
//						temp.peak[0].mean[i] += v2.point[0].value[j];
					d2_match_axis[match_counter]=i;
					d1_match_axis[match_counter]=j;
					match_counter++;
				}
			}
		}
		
			cout << "Variance calculating...";
			Matrix2d d1_variance_matrix;
			d1_variance_matrix << d1.peak[0].variance(d1_match_axis[0],d1_match_axis[0])
								, d1.peak[0].variance(d1_match_axis[0],d1_match_axis[1])
								, d1.peak[0].variance(d1_match_axis[1],d1_match_axis[0])
								, d1.peak[0].variance(d1_match_axis[1],d1_match_axis[1]);
			Matrix2d d2_variance_matrix;
			d2_variance_matrix << d2.peak[0].variance(d2_match_axis[0],d2_match_axis[0])
								, d2.peak[0].variance(d2_match_axis[0],d2_match_axis[1])
								, d2.peak[0].variance(d2_match_axis[1],d2_match_axis[0])
								, d2.peak[0].variance(d2_match_axis[1],d2_match_axis[1]);
			d1_variance_matrix = d1_variance_matrix + d2_variance_matrix;
			temp.peak[0].variance(d1_match_axis[0],d1_match_axis[0]) = d1_variance_matrix(0,0);
			temp.peak[0].variance(d1_match_axis[0],d1_match_axis[1]) = d1_variance_matrix(0,1);
			temp.peak[0].variance(d1_match_axis[1],d1_match_axis[0]) = d1_variance_matrix(1,0);
			temp.peak[0].variance(d1_match_axis[1],d1_match_axis[1]) = d1_variance_matrix(1,1);
/*			for(int l=0;l<d1.peak[0].axis.size();l++){
				if(d2.peak[0].axis[k]==d1.peak[0].axis[l]){
					cout << "convolve..\n";
					temp.peak[0].axis[l]=d1.peak[0].axis[k];
					temp.peak[0].mean[l]=d1.peak[0].mean[k]+d2.peak[0].mean[l];
					temp.peak[0].variance(l,l)=d1.peak[0].variance(k,k)+d2.peak[0].variance(l,l);
				}
			}
*/
	}
// above is for 2x2 variance calculation
//
// if distribution is not 2 axis, do below
// Examine if the axis match up 
	else{
	for(int k=0;k<d2.peak[0].axis.size();k++){
// if match axis is found, then convolve		
		if (binary_search (d1.peak[0].axis.begin(),d1.peak[0].axis.end(), d2.peak[0].axis[k])){
			for(int l=0;l<d1.peak[0].axis.size();l++){
				if(d2.peak[0].axis[k]==d1.peak[0].axis[l]){
					cout << "convolve..\n";
					temp.peak[0].axis[l]=d1.peak[0].axis[k];
					temp.peak[0].mean[l]=d1.peak[0].mean[k]+d2.peak[0].mean[l];
					temp.peak[0].variance(l,l)=d1.peak[0].variance(k,k)+d2.peak[0].variance(l,l);
				}
			}
		}
// if match not found, add this axis
		else{
			cout << "add..\n";
			temp.peak[0].axis.push_back(d2.peak[0].axis[k]);
			temp.peak[0].mean.conservativeResize(temp.peak[0].axis.size());
			temp.peak[0].mean(temp.peak[0].axis.size()-1)=d2.peak[0].mean[k];
			temp.peak[0].variance.conservativeResize(temp.peak[0].axis.size(),temp.peak[0].axis.size());
			for (int m=0;m<temp.peak[0].axis.size();m++)
			{
				temp.peak[0].variance(m, temp.peak[0].axis.size()-1)=0;
				temp.peak[0].variance(temp.peak[0].axis.size()-1, m)=0;
			}
			temp.peak[0].variance(temp.peak[0].axis.size()-1,temp.peak[0].axis.size()-1)=d2.peak[0].variance(k,k);
		}
	}
	}

        // create a new object to pass back as the result
        norm_dist *pNew = new norm_dist();
        *pNew = temp;
//        _G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

Vector *convolveNV(double p2, Vector d1)
{
/*	Warning!! This is a very raw calculation, 
	might not be suitable for more than 1 axis!! */
	stringstream s;
	s << p2;
	string result_name = "(" + s.str() + "+" + d1.name + ")";
	Vector temp(result_name);
	temp.name=result_name;
	temp.point[0] = d1.point[0];
	for(int i=0;i<temp.point[0].axis.size();i++)
		temp.point[0].value[i] += p2;

	// create a new object to pass back as the result
	Vector *pNew = new Vector();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
Vector *deconvolveNV(double p2, Vector d1)
{
/*	Warning!! This is a very raw calculation, 
	might not be suitable for more than 1 axis!! */
	stringstream s;
	s << p2;
	string result_name = "("+s.str() + "-" + d1.name+")";
	Vector temp(result_name);
	temp.point[0] = d1.point[0];
	for(int i=0;i<temp.point[0].axis.size();i++)
		temp.point[0].value[i] = temp.point[0].value[i]-p2;

	// create a new object to pass back as the result
	Vector *pNew = new Vector();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}

Vector *deconvolveVV(Vector vect1, Vector vect2)
{
	string result_name = "("+vect1.name + "-" + vect2.name+")";
	Vector temp(result_name);
	temp.point[0] = vect1.point[0];

// Examine if the axis match up 
	for(int k=0;k<vect2.point[0].axis.size();k++){
// if match axis is found, then convolve		
		if (binary_search (vect2.point[0].axis.begin(),vect2.point[0].axis.end(), vect1.point[0].axis[k])){
			for(int l=0;l<vect1.point[0].axis.size();l++){
				if(vect2.point[0].axis[k]==vect1.point[0].axis[l]){
					cout << "subtracting..\n";
					temp.name=result_name;
					temp.point[0].axis[l]=vect1.point[0].axis[k];
					temp.point[0].value[l]=vect1.point[0].value[k]-vect2.point[0].value[l];
				}
			}
		}
// if match not found, add this axis
		else{
			cout << "axis not found, adding..\n";
			temp.point[0].axis.push_back(vect2.point[0].axis[k]);
			temp.point[0].value.conservativeResize(temp.point[0].axis.size());
			temp.point[0].value(temp.point[0].axis.size()-1)=-vect2.point[0].value[k];
		}
	}
	// create a new object to pass back as the result
	Vector *pNew = new Vector();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}


Vector *convolveVV(Vector vect1, Vector vect2)
{
	bool match_flag = false;
	string result_name = "("+vect1.name + "+" + vect2.name+")";
	Vector temp(result_name);
	temp.name = result_name;
	temp.point[0] = vect1.point[0];
	// looking for match axis and convolve, create a new object to pass back as the result
cout << "Vector '" << result_name << "'copied.\n";
	for(int i=0;i<vect2.point[0].axis.size();i++){
		for(int j=0;j<temp.point[0].axis.size();j++){
			if(temp.point[0].axis[j]==vect2.point[0].axis[i]){
				temp.point[0].value[i] += vect2.point[0].value[j];
				match_flag = true;
cout << "Vector '" << vect2.name << "'added.\n";
			}
		}
		if(match_flag == false){
			temp.point[0].axis.push_back(vect2.point[0].axis[i]);
			temp.point[0].value.conservativeResize(temp.point[0].axis.size());
			temp.point[0].value(temp.point[0].axis.size()-1)=vect2.point[0].value[i];
			//temp.point[0].value.push_back(vect2.point[0].value[i]);
		}
		match_flag = false;
	}

temp.print();
// create a new object to pass back as the result
	Vector *pNew = new Vector();
	*pNew = temp;
//	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
/*----------------------------------------------------------------------

Extern "C" definitions

----------------------------------------------------------------------*/ 

static int _G_dindex=0;
// mimic the behavior of an iterator on the array list
// no boundary checking!

extern "C" void *DIF_getnextfromlist(void *list, int listindex, char type){
	if (type=='d'){
		MoG *dlist;
		dlist = (MoG *)list;
		//printf("Passing MoG pointer.\n");
		//dlist[listindex].print();
		return (void *) &(dlist[listindex]);
	}
	if (type=='x'){
		Vector *dlist;
		dlist = (Vector *)list;
		//printf("Passing vector pointer.\n");
		//dlist[listindex].print();
		return (void *) &(dlist[listindex]);
	}
	if (type=='a'){
		array *dlist;
		dlist = (array *)list;
		//printf("DIF_getnext; Passing array pointer.\n");
		//dlist[listindex].print();
		return (void *) &(dlist[listindex]);
	}
	if (type=='m'){
		MoG *dlist;
		dlist = (MoG *)list;
		//printf("Passing MoG pointer.\n");
		//dlist[listindex].print();
		return (void *) &(dlist[listindex]);
	}
}

extern "C" void DIF_CondConvolve(void *dist1, void *dist2, void *distresult){
  if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveMM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      CondConvolve( *((MoG *)dist1), *((MoG *)dist2));
 return;
  }
}
extern "C" void DIF_convolveMZ(void *dist1, void *dist2, void *distresult){
  if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveMM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      convolveMZ( *((MoG *)dist1), *((MoG *)dist2));
      /*???????????????????????*/
 return;
  }
}
extern "C" void DIF_convolveMM(void *dist1, void *dist2, void *distresult){
  if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveMM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      convolveMM( *((MoG *)dist1), *((MoG *)dist2));
 return;
  }
}
extern "C" void DIF_deconvolveMM(void *dist1, void *dist2, void *distresult){
  if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveMM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      deconvolveMM( *((MoG *)dist1), *((MoG *)dist2));
 return;
  }
}
extern "C" void DIF_disjunctMM(void *dist1, void *dist2, void *distresult){
  if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to disjunctMM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      disjunctMM( *((MoG *)dist1), *((MoG *)dist2));
 return;
  }
}
extern "C" void DIF_disjunctZZ(void *dist1, void *dist2, void *distresult){
  if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to disjunctMM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      disjunctZZ( *((MoG *)dist1), *((MoG *)dist2));
 return;
  }
}
// the interface function to do a convolution
extern "C" void DIF_convolveVD(void *vect1, void *dist2, void *distresult){
  if (vect1==NULL) {
    printf("DLIBINTERFACE error: null vector pointer sent to convolveVD\n");
    return;
  }
  else if (dist2==NULL) {
    printf("DLIBINTERFACE error: null dist pointer sent to convolveVD\n");
    return;
  }
  else if (distresult==NULL) {
    printf("DLIBINTERFACE error: null result pointer sent to convolveVD\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      convolveVD( *((Vector *)vect1), *((norm_dist *)dist2));
 return;
  }
}
extern "C" void DIF_convolveVM(void *vect1, void *dist2, void *distresult){
  if (vect1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveVM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      convolveVM( *((Vector *)vect1), *((MoG *)dist2));
 return;
  }
}
extern "C" void DIF_deconvolveDV(void *vect1, void *dist2, void *distresult){
  if (vect1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveVD\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      deconvolveDV( *((Vector *)vect1), *((norm_dist *)dist2));
 return;
  }
}

extern "C" void DIF_deconvolveVD(void *vect1, void *dist2, void *distresult){
  if (vect1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveVD\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      deconvolveVD( *((Vector *)vect1), *((norm_dist *)dist2));
 return;
  }
}
extern "C" void DIF_deconvolveMV(void *dist2, void *vect1, void *distresult){
  if (vect1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveVD\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      deconvolveMV( *((Vector *)vect1), *((MoG *)dist2));
 return;
  }
}

extern "C" void DIF_deconvolveVM(void *vect1, void *dist2, void *distresult){
  if (vect1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveVD\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      deconvolveVM( *((Vector *)vect1), *((MoG *)dist2));
 return;
  }
}
extern "C" void DIF_convolveDD(void *dist1, void *dist2, void *distresult){
  if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveDD\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      convolveDD( *((norm_dist *)dist1), *((norm_dist *)dist2));
 return;
  }
}
/*
extern "C" void DIF_convolveDN(void *dist1, double point, void *distresult){
  if (dist1==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveDD\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      convolveDN( *((norm_dist *)dist1), point);
 return;
  }
}
*/
extern "C" void DIF_convolveNV(double point, void *vect2, void *vectresult){
  if (vect2==NULL || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveNV\n");
    return;
  }
  else {
    *(Vector **)vectresult = 
      convolveNV( point, *((Vector *)vect2));
 return;
  }
}

extern "C" void DIF_deconvolveNV(double point, void *vect2, void *vectresult){
  if (vect2==NULL || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveNV\n");
    return;
  }
  else {
    *(Vector **)vectresult = 
      deconvolveNV( point, *((Vector *)vect2));
 return;
  }
}
/*
extern "C" void DIF_convolveND(double point, void *dist2, void *distresult){

  if (dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveND\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      convolveND( point, *((norm_dist *)dist2));
 return;
  }
}*/
extern "C" void DIF_convolveVV(void *vect1, void *vect2, void *vectresult){
  if (vect1==NULL || vect2==NULL || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to convolveVV\n");
    return;
  }
  else {
    *(Vector **)vectresult = 
      convolveVV( *((Vector *)vect1), *((Vector *)vect2));
 return;
  }
}
extern "C" void DIF_deconvolveVV(void *vect1, void *vect2, void *vectresult){
  if (vect1==NULL || vect2==NULL || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to deconvolveVV\n");
    return;
  }
  else {
    *(Vector **)vectresult = 
      deconvolveVV( *((Vector *)vect1), *((Vector *)vect2));
 return;
  }
}
extern "C" void DIF_scaleD(double scale, void *dist, void *distresult){
  /*printf("Error: scaling not defined yet\n");
  *(norm_dist **)distresult=(norm_dist *)dist;
  */
   if (dist==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleD\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      scaleD( scale, *((norm_dist *)dist));
  return;
  }
}
extern "C" void DIF_scaleM(double scale, void *dist, void *distresult){
  /*printf("Error: scaling not defined yet\n");
  *(norm_dist **)distresult=(norm_dist *)dist;
  */
   if (dist==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleM\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      scaleM( scale, *((MoG *)dist));
  return;
  }
}
extern "C" void DIF_scaleZ(double scale, void *dist, void *distresult){
   if (dist==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleZ\n");
    return;
  }
  else {
    *(MoG **)distresult = 
      scaleZ( scale, *((MoG *)dist));
  return;
  }
}

//
// **** DML Aug 31 2013 modified this to fix crash!!!!
// remove 3rd argument

extern "C" void DIF_scaleVV(void *vect1, void *vect2){//, void *vectresult){
  if (vect1 == NULL || vect2==NULL){// || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleD\n");
    return;
  }
  else {
    //    *(Vector **)vectresult = 
      scaleVV( *((Vector *)vect1) , *((Vector *)vect2));
  return;
  }
}
extern "C" void DIF_scaleVM(void *vect, void *dist, void *vectresult){
   if (vect == NULL || dist==NULL || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleVZ\n");
    return;
  }
  else {
    *(Vector **)vectresult = 
      scaleVM( *((Vector *)vect) , *((MoG *)dist));
  return;
  }
}
extern "C" void DIF_scaleVZ(void *vect, void *dist, void *vectresult){
   if (vect == NULL || dist==NULL || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleVZ\n");
    return;
  }
  else {
    *(Vector **)vectresult = 
      scaleVZ( *((Vector *)vect) , *((MoG *)dist));
  return;
  }
}
extern "C" double DIF_dotVV(void *vect1, void *vect2){
   double dot_product=0;
   if (vect1 == NULL || vect2==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleVV\n");
    return dot_product;
  }
  else {
    dot_product = 
      dotVV( *((Vector *)vect1) , *((Vector *)vect2));
  return dot_product;
  }
}

extern "C" double DIF_dotMM(void *dist1, void *dist2){
   double dot_product=0;
   if (dist1 == NULL || dist2==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleVV\n");
    return dot_product;
  }
  else {
    dot_product = 
      dotMM( *((MoG *)dist1) , *((MoG *)dist2));
  return dot_product;
  }
}
/*
extern "C" double DIF_scaleMM(void *dist1, void *dist2){
   double dot_product=0;
   if (dist1 == NULL || dist2==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleVV\n");
    return dot_product;
  }
  else {
    dot_product = 
      scaleMM( *((MoG *)dist1) , *((MoG *)dist2));
  return dot_product;
  }
}*/
extern "C" void DIF_scaleMM(void * dist1, void *dist2, void *distresult){
   if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to rotateVM\n");
    return;
  }
  else {
	   printf("Passing pointers...\n");
    *(MoG **)distresult = 
      scaleMM(*((MoG *)dist1), *((MoG *)dist2));
  return;
  }
}
extern "C" void DIF_scaleV(double scale, void *vect, void *vectresult){
  /*printf("Error: scaling not defined yet\n");
  *(norm_dist **)distresult=(norm_dist *)dist;
  */
   if (vect==NULL || vectresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleV\n");
    return;
  }
  else {
    *(Vector **)vectresult = 
      scaleV( scale, *((Vector *)vect));
  return;
  }
}
extern "C" void DIF_scaleVD(void * vect, void *dist, void *distresult){
  /*printf("Error: scaling not defined yet\n");
  *(norm_dist **)distresult=(norm_dist *)dist;
  */
   if (vect==NULL || dist==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to scaleD\n");
    return;
  }
  else {
    *(norm_dist **)distresult = 
      scaleVD(*((Vector *)vect), *((norm_dist *)dist));
  return;
  }
}
extern "C" void DIF_rotateVD(void * vect, void *dist, void *distresult){
   if (vect==NULL || dist==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to rotateVD\n");
    return;
  }
  else {
	   printf("Passing pointers...\n");
    *(norm_dist **)distresult = 
      rotateVD(*((Vector *)vect), *((norm_dist *)dist));
  return;
  }
}
extern "C" void DIF_rotateVM(void * vect, void *dist, void *distresult){
   if (vect==NULL || dist==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to rotateVM\n");
    return;
  }
  else {
	   printf("Passing pointers...\n");
    *(MoG **)distresult = 
      rotateVM(*((Vector *)vect), *((MoG *)dist));
  return;
  }
}
extern "C" void DIF_rotateMM(void * dist1, void *dist2, void *distresult){
   if (dist1==NULL || dist2==NULL || distresult==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to rotateVM\n");
    return;
  }
  else {
	   printf("Passing pointers...\n");
    *(MoG **)distresult = 
      rotateMM(*((MoG *)dist1), *((MoG *)dist2));
  return;
  }
}
extern "C" void DIF_addD(double scale, void *dist, void *distresult){
  printf("Error: addition not defined yet\n");
  *(norm_dist **)distresult=(norm_dist *)dist;
  return;
}
extern "C" void DIF_addV(double scale, void *vect, void *vectresult){
  printf("Error: addition not defined yet\n");
  *(Vector **)vectresult=(Vector *)vect;
  return;
}
extern "C" void DIF_printD(void *dist){
  ((norm_dist *)dist)->print();
  return;
}
extern "C" void DIF_printM(void *dist){
  ((MoG *)dist)->print();
  return;
}
extern "C" void DIF_printV(void *vect){
  ((Vector *)vect)->print();
  return;

}extern "C" void DIF_printA(void *arr){
  ((array *)arr)->print();
  return;
}
/*
double ltrD(norm_dist dist, Vector vect)
{
	double pNew=0;
	cout << "Lesser than:\n";
	pNew=dist.ltr(vect);
	cout << "\n-->" << pNew << endl;
	return pNew;
} 
extern "C" double DIF_ltrD(void *dist, void *vect){
   double temp=0;
   if (dist == NULL || vect==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to LTR\n");
    return temp;
  }
  else {
    temp = ltrD( *((norm_dist *)dist) , *((Vector *)vect));
	printf("in DIF_ltrD: %f\n", temp);
	return temp;
  }
}
double gtrD(norm_dist dist, Vector vect)
{
	double pNew=0;
	cout << "Lesser than:\n";
	pNew=dist.gtr(vect);
	cout << "\n-->" << pNew << endl;
	return pNew;
} 
extern "C" double DIF_gtrD(void *dist, void *vect){
   double temp=0;
   if (dist == NULL || vect==NULL) {
    printf("DLIBINTERFACE error: null pointer sent to gtr\n");
    return temp;
  }
  else {
    temp = gtrD( *((norm_dist *)dist) , *((Vector *)vect));
	printf("in DIF_gtrD: %f\n", temp);
	return temp;
  }
}
*/

extern "C"	void DIF_ltrD(void *dist, void *vect, double & answer){
	answer=((norm_dist *)dist)->ltr((Vector *)vect);
	return;
}
extern "C"	void DIF_gtrD(void *dist, void *vect, double & answer){
	answer=((norm_dist *)dist)->gtr((Vector *)vect);
	return;
}
extern "C"	void DIF_eqD(void *dist, void *vect, double & answer){
	answer=((norm_dist *)dist)->eq((Vector *)vect);
	return;
}
extern "C"	void DIF_neqD(void *dist, void *vect, double & answer){
	answer=((norm_dist *)dist)->neq((Vector *)vect);
	return;
}

// does garbage collection of any temporary nodes created
// during matheval processing.
extern "C" void DIF_dogarbagecollection(){
/*
  vector<norm_dist*>::iterator di;
  for (di=_G_dynamic_memory_list.begin();
       di!=_G_dynamic_memory_list.end(); di++) delete(*di);
*/
 _G_dindex = 0;
}

/*
norm_dist *convolveDN(norm_dist d1, double p2)
{
//	Warning!! This is a very raw calculation, 
//	might not be suitable for more than 1 axis!!
	stringstream s;
	s make all<< p2;
	string result_name = d1.distribution_name + "+" + s.str();
	norm_dist temp(result_name);
	temp.peak[0] = d1.peak[0];
	for(int i=0;i<temp.peak[0].axis.size();i++)
		temp.peak[0].mean[i] += p2;

	// create a new object to pass back as the result
	norm_dist *pNew = new norm_dist();
	*pNew = temp;
	_G_dynamic_memory_list.push_back(pNew); // record the pointer for later garbage collection
	return pNew;
}
*/
extern "C" void * tss_condD(void *op, void *arg1, void *arg2)
{
	MoG * ans;
	int cond_mode = *(double *)op;
	/*
  if (op!=0 && arg1!=0 && arg2!=0) {
		printf("Calling TSS condD.. \nop=%i\n\narg1=",cond_mode);
		DIF_printM(arg1);
		printf("arg2=");
		DIF_printV(arg2);
	} else printf("tss_condD -- args missing\n");
	*/
/*	if (cond_mode == 0){ 		// do less than
	  printf("-EQ-\n");
	  double ans = DIF_eqM(arg1, arg2);
	}
  /*else if (cond_mode == 1){	// do greater than
	  printf("-NEQ-\n");
	  MoG ans = DIF_neqM(arg1, arg2);
  }
  else */
	if (cond_mode == 1){
		printf("-GTR-\n");
		string name_str = "CondD(GTR,"+((MoG*)arg1)->distribution_name+","+((Vector*)arg2)->name+")";
		ans = new MoG(name_str);
		ans->gtr(((MoG*)arg1),((Vector*)arg2));
		ans->print();
		return ((void *)&ans[0]);
	}
	else if (cond_mode == 2){
		printf("-LTR-\n");
		string name_str = "CondD(LTR,"+((MoG*)arg1)->distribution_name+","+((Vector*)arg2)->name+")";
		ans = new MoG(name_str);
		ans->ltr(((MoG*)arg1),((Vector*)arg2));
		ans->print();
		return ((void *)&ans[0]);
	}
	//else if (cond_mode == 3){
		//printf("-GTE-\n");
		//string name_str = "CondD(GTE,"+((MoG*)arg1)->distribution_name+","+((Vector*)arg2)->name+")";
		//ans = new MoG(name_str);
		//ans->gte(((MoG*)arg1),((Vector*)arg2));
		//ans->print();
		//return ((void *)&ans[0]);
	//}
	//else if (cond_mode == 4){
		//printf("-LTE-\n");
		//string name_str = "CondD(LTE,"+((MoG*)arg1)->distribution_name+","+((Vector*)arg2)->name+")";
		//ans = new MoG(name_str);
		//ans->lte(((MoG*)arg1),((Vector*)arg2));
		//ans->print();
		//return ((void *)&ans[0]);
	//}
	else if (cond_mode == 5){
		printf("-Freeze-\n");
		string name_str = "CondD(FRZ,"+((MoG*)arg1)->distribution_name+","+((Vector*)arg2)->name+")";
		ans = new MoG(name_str);
		ans->setFreeze(((MoG*)arg1), ((Vector*)arg2));
		ans->print();
		return ((void *)&ans[0]);
	}
	else if (cond_mode == 100){
		printf("-InRoom-\n");
		string name_str = "CondD(InRoom,"+((MoG*)arg1)->distribution_name+")";//+((Vector*)arg2)->name+")";
		ans = new MoG(name_str);
		ans->InRoom((MoG*)arg1);//,((Vector*)arg2));
		ans->print();
		return ((void *)&ans[0]);
	}
	else if (cond_mode == 200){
		printf("-NotInRoom-\n");
		string name_str = "CondD(NotInRoom,"+((MoG*)arg1)->distribution_name+")";//+((Vector*)arg2)->name+")";
		ans = new MoG(name_str);
		ans->NotInRoom((MoG*)arg1);//,((Vector*)arg2));
		ans->print();
		return ((void *)&ans[0]);
	}
        /*else if (cond_mode == 300){
		printf("-OverlapInRoom-\n");
		string name_str = "CondD(OverlapInRoom,"+((MoG*)arg1)->distribution_name+ "," +((Vector*)arg2)->name+")";
		ans = new MoG(name_str);
		ans->OverlapInRoom((MoG*)arg1, (MoG*)arg2);//,((Vector*)arg2));
		ans->print();
		return ((void *)&ans[0]);
	}*/
  else if (cond_mode == 1000){
		printf("-Overlap-\n");
		string name_str = "CondD(Overlap,"+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
		ans = new MoG(name_str);
		ans->Overlap((MoG*)arg1, (MoG*)arg2);
		ans->print();
		return ((void *)&ans[0]);
	}
	else if (cond_mode == 1001){
		printf("-NoOverlap-\n");
		string name_str = "CondD(NoOverlap,"+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
		ans = new MoG(name_str);
		ans->NoOverlap((MoG*)arg1, (MoG*)arg2);
		ans->print();
		return ((void *)&ans[0]);
	}
        
	else
		printf("Argument input invalid!! Please check the equation. \n");
		exit(1);
}

/* New function for PDIFF */
static float tss_pdiff_return=0;

extern	"C" void *tss_pdiff(void *arg1, void *arg2, void *op){
  double ans=0;
  int pdiff_mode = *(double *)op;
  ans = 0.5; // just a random number I picked

  if (op!=0 && arg1!=0 && arg2!=0) {
    printf("Calling TSS PDIFF.. (op=%d)\n",pdiff_mode);
	} else printf("TSS PDIFF -- args missing\n");
  if (op!=0 && arg1!=0 && arg2!=0) {
    printf("Calling TSS PDIFF.. (op=%d)\n",pdiff_mode);
    double tempValue = ((MoG*)arg1)->OverlapInRoom(((MoG*)arg2));
    cout << "InRoom NonOverlap = " << tempValue << endl;
    ans = tempValue;
  } 
  else printf("TSS PDIFF -- args missing\n");
  /* do some processing here */

  tss_pdiff_return = ans;
  return (void *) &tss_pdiff_return;
}



/* DMl SEPT 2013, 3 new functions for MOGS */

/* returns MoG with means that are unit vectors of argument MoG's means */

extern "C"  void *DIF_mf_unit(void *arg1) {
  MoG *ans,*d1;
  string name_str = "unit("+((MoG*)arg1)->distribution_name+")";
  cout << "UNIT function call: "<<name_str<<endl;
  d1 = ((MoG *)arg1);
  d1->print();
  ans = new MoG(name_str,d1->peak.size());
  *ans = *d1;
  ans->distribution_name=name_str;

  for (int i=0; i<d1->peak.size(); i++)  {// each peak has mu  normalized
    double sum=0;
    for (int j=0; j<d1->peak[i].axis.size(); j++) {
      sum += d1->peak[i].mean[j]*d1->peak[i].mean[j];
    }
    double sqrtsum=(sum>0)?(sqrt(sum)):0.0; 
    for (int j=0; j<d1->peak[i].axis.size(); j++) {
      ans->peak[i].mean[j] = (sqrtsum>0)?(d1->peak[i].mean[j]/sqrtsum):0.0;
    }
    ans->peak[i].weight = d1->peak[i].weight;
    ans->peak[i].peakID = d1->peak[i].peakID;
    ans->peak[i].variance.setZero(); // ??????? is it the right thing to do ????? JL Oct 31
    cout << "in mf_unit: has variance.setZero(), is it the right thing to do?\n";
  } 
  ans->check();
  ans->print();
  //  cerr<<name_str<<"="<<ans->peak[0].mean<<endl;
  return ((void *)&ans[0]);

}

/* returns MoG with means that are XY plane angle (all axis set to this value) of argument MoG's means */

extern "C" void *DIF_mf_angle(void *arg1) {
  MoG *ans,*d1;
  string name_str = "angle("+((MoG*)arg1)->distribution_name+")";
  cout << "ANGLE functional call: "<<name_str<<endl;
  d1 = ((MoG *)arg1);
  //d1->print();
  ans = new MoG(name_str,d1->peak.size());
  *ans = *d1;
  ans->distribution_name=name_str;

  for (int i=0; i<d1->peak.size(); i++)  {// each peak has mean all values set to angle
    double sum=0;
    for (int j=0; j<d1->peak[i].axis.size(); j++) {
      sum += d1->peak[i].mean[j];
    }
    double ang = sum>0?atan2(d1->peak[i].mean[1]/sum, d1->peak[i].mean[0]/sum):0.0;
    for (int j=0; j<d1->peak[i].axis.size(); j++) {
      ans->peak[i].mean[j] = ang;
    }
    ans->peak[i].variance.setZero();
  } 
  //  ans->print();
  return ((void *)&ans[0]);
}


extern "C" void *DIF_mf_cut(void *arg1) {
  MoG *ans,*d1;
  string name_str = "cut("+((MoG*)arg1)->distribution_name+")";
  cout << "CUT function call:  "<<name_str<<endl;
  d1 = ((MoG *)arg1);
  d1->print();
  ans = new MoG(name_str,d1->peak.size());
  *ans = *d1;
  ans->distribution_name=name_str;

  for (int i=0; i<d1->peak.size(); i++)  {
    for (int j=0; j<d1->peak[i].axis.size(); j++) {      //DO SOMETHING
      ans->peak[i].mean[j] = (d1->peak[i].mean[j] < 1) ? d1->peak[i].mean[j]:1;
    }
  }
  ans->print();
  return ((void *)&ans[0]);
}



/* returns MoG with means that are L2 length (all set to that value) of argument MoG's means */


extern "C" void *DIF_mf_length(void *arg1) {
  MoG *ans,*d1;
  string name_str = "length("+((MoG*)arg1)->distribution_name+")";
  cout << "LENGTH function call:  "<<name_str<<endl;
  d1 = ((MoG *)arg1);
  d1->print();
  ans = new MoG(name_str,d1->peak.size());
  *ans = *d1;
  ans->distribution_name=name_str;

  for (int i=0; i<d1->peak.size(); i++)  {// each peak has mean all values set to length
    double sum=0;
    for (int j=0; j<d1->peak[i].axis.size(); j++) {
      sum += d1->peak[i].mean[j] * d1->peak[i].mean[j] ;
    }
    double sqrtsum=(sum>0)?(sqrt(sum)):0.0; 
    cout<<"LENGTH pk"<<i<<" sum="<<sum<<" sqrt sum="<<sqrtsum<<endl;
    for (int j=0; j<d1->peak[i].axis.size(); j++) {
      ans->peak[i].mean[j] = sqrtsum;
    }
    ans->peak[i].variance.setZero();
  }
  ans->print();
  return ((void *)&ans[0]);
}

extern "C" void *DIF_mf_union(void *arg1, void *arg2){
  MoG *ans, *d1, *d2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  int result_size = d1->peak.size() + d2->peak.size();
  string name_str = "union("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout << "UNION function call" << endl;
  ans = new MoG(name_str,result_size);

  
  vector<distribution>::iterator ans_it = ans->peak.begin();
  for (vector<distribution>::iterator lhs_it = d1->peak.begin(); lhs_it != d1->peak.end(); lhs_it++){
    *ans_it = *lhs_it;
    //hack .. make all colors = 0 -dagan
    ans_it->peakID = 0;
    ans_it->weight = lhs_it->weight * d1->peak.size() / result_size;
    if (ans_it != ans->peak.end())
      ans_it++;
  }
  for (vector<distribution>::iterator rhs_it = d2->peak.begin(); rhs_it != d2->peak.end(); rhs_it++){
    *ans_it = *rhs_it;
    //hack .. make all colors = 0 -dagan
    ans_it->peakID = 0;
    ans_it->weight = rhs_it->weight * d2->peak.size() / result_size;
    if (ans_it != ans->peak.end())
      ans_it++;
  }
  

    
  cout<<"UNION result is:\n";
  ans->print();
  //return ((void *)&ans[0]);
  return ((void *)&ans[0]);
  
  
}
/*
 rot_sign : a helper function to determine coefficients for uncertainty
 ie . rotational uncertainty distributions are gathered from GATech data
 based on 90 degree COUNTERCLOCKWISE turns. We derive the distributions but,
 generally, they tell you how to transform your uncertainty when you change headings.
 We want to apply the mirror of that transformation (ONLY ACROSS THE X AXIS)
 if your heading change was a 'right turn'.
 
 we will assume 'right turn' means that, from old heading to new heading, the
 clockwise angle difference is shorter than the counterclockwise angle distance.

 since only using for rotational uncertainty and the uncertainty MoG gets rotated
 only going to flip across the x axis (mean[1]) ... the coefficient for mean[0] = 1

 if moving to 3d spaces .. this will not work
 */
extern "C" void *DIF_mf_rot_sign(void *arg1, void *arg2){

  MoG *ans,*d1,*d2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  string name_str = "rot_sign("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout<<" ROT_SIGN function call: "<<name_str<<endl;
  int d1PeakSize = d1->peak.size(), d2PeakSize = d2->peak.size();
  ans = new MoG(name_str,max(d1PeakSize, d1PeakSize*d2PeakSize));
  d1->print();
  d2->print();
  //*ans = *d2;
  ans->distribution_name=name_str;

  for(int i=0;i<d1PeakSize;i++){
    for(int j=0;j<d2PeakSize;j++){
      int n = j+i*d2PeakSize; // index of temp peak
      if(d1->peak[i].freeze == 0 && d2->peak[j].freeze == 0){
	cout << "d1.peak[" << i << "](peakID=" << d1->peak[i].peakID << ") "
	     << " vs d2.peak[" << j << "](peakID=" << d2->peak[j].peakID << ")...\n";

	if(d1->peak[i].peakID == d2->peak[j].peakID){
	  cout << "Match!! in same peaks" << endl;
	  ans->peak[n] = d2->peak[j];
	  ans->peak[n].variance.setZero();
	  ans->peak[n].mean[0] = 1;
	  cout <<     atan2(d1->peak[i].mean[1], d1->peak[i].mean[0])	- atan2(d2->peak[j].mean[1], d2->peak[j].mean[0]) << endl;


	  ans->peak[n].mean[1] =( ( atan2(d1->peak[i].mean[1], d1->peak[i].mean[0])
	  			    - atan2(d2->peak[j].mean[1], d2->peak[j].mean[0])) < 0 ) ? -1 : 1;

	  cout << "result is mean=("<<ans->peak[n].mean[0]<<", "<<ans->peak[n].mean[1]<<")"<<endl;
	}
	else if(d1->peak[i].peakID >= 256){
	  cout << "Match!! in lhs 256" << endl;

	  ans->peak[n] = d2->peak[j];
	  ans->peak[n].variance.setZero();
	  ans->peak[n].mean[0] = 1;
	  ans->peak[n].mean[1] =( ( atan2(d1->peak[i].mean[1], d1->peak[i].mean[0])
				    - atan2(d2->peak[j].mean[1], d2->peak[j].mean[0])) < 0 ) ? -1 : 1;



	}
	else if(d2->peak[j].peakID >= 256){
	  cout << "Match!! in rhs 256" << endl;
	  ans->peak[n] = d2->peak[j];
	  ans->peak[n].variance.setZero();
	  ans->peak[n].mean[0] = 1;
	  ans->peak[n].mean[1] =( ( atan2(d1->peak[i].mean[1], d1->peak[i].mean[0])
				    - atan2(d2->peak[j].mean[1], d2->peak[j].mean[0])) < 0 ) ? -1 : 1;

	}
	
	else{ ans->peak[n] = d2->peak[j]; ans->peak[n].weight = 0 ; }	//what case does this cover? non-matching colors?
      }// end of both not frozen

      else if(d1->peak[i].freeze ==1 || d2->peak[j].freeze==1){
	cout << "Seeing pre-freezer, peak freeze.\n";
      }
      else if (d2->peak[j].freeze == 2){
	cout << "Peak frozen.\n";
        //  temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
      }
      else if (d1->peak[i].freeze == 2){
	cout << "Peak frozen.\n";
        //  temp.peak[n].weight = d2.peak[j].weight * d1.peak[i].weight;
      }
      else{
	cerr << "rot_sign error due to freeze problem\n";
	d1->print();
	d2->print();
	exit(1);
      }
    }
  }

  ans->check();
  cout<<"ROT_SIGN result is:\n";
  ans->print();
  return ((void *)&ans[0]);

}

//return a MoG s.t. : sgn1(x) = 0 if x < 1  |  1 otherwise
extern "C" void *DIF_mf_sgn1(void *arg){
  MoG *ans,*d;
  d = ((MoG*)arg);
  
  string name_str = "sgn1("+((MoG*)arg)->distribution_name+")";
  cout<<" SGN1 function call: "<<name_str<<endl;

  int PeakSize = d->peak.size();
  d->print();
  ans = new MoG(name_str,d->peak.size());
  *ans = *d;
  ans->distribution_name=name_str;

  for (int i=0; i<d->peak.size(); i++){
    for (int j=0; j<d->peak[i].axis.size(); j++)
      ans->peak[i].mean[j] = (fabs(d->peak[i].mean[j]) < 1) ? 0 : 1;
    ans->peak[i].variance.setZero();
  }
  
  ans->print();
  return ((void *)&ans[0]);
}

extern "C" void *DIF_mf_flip_sgn1(void *arg){
  MoG *ans,*d;
  d = ((MoG*)arg);
  
  string name_str = "flip_sgn1("+((MoG*)arg)->distribution_name+")";
  cout<<" FLIP_SGN1 function call: "<<name_str<<endl;

  int PeakSize = d->peak.size();
  d->print();
  ans = new MoG(name_str,d->peak.size());
  *ans = *d;
  ans->distribution_name=name_str;

  for (int i=0; i<d->peak.size(); i++){
    for (int j=0; j<d->peak[i].axis.size(); j++)
      ans->peak[i].mean[j] = (fabs(d->peak[i].mean[j]) < 1) ? 1 : 0;
    ans->peak[i].variance.setZero();
  }
  
  ans->print();
  return ((void *)&ans[0]);
}

extern "C" void *DIF_mf_pwconv(void *arg1, void *arg2){

  MoG *ans,*d1,*d2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  string name_str = "pwconv("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout<<" PWCONV function call: "<<name_str<<endl;

  int d1PeakSize = d1->peak.size(), d2PeakSize = d2->peak.size();;

  //all cases disjoint, extra case provides commutativity
  //colors and weights and variances inherited from LHS call to *convolve_single_peak
  //if one side has 1 peak and other has > 1 : send 'other' as first param to *convolve_single_peak
  if (d1PeakSize == d2PeakSize){
    ans = new MoG(name_str,d1PeakSize);
    for (int i=0; i<d1PeakSize; i++){
      //ans->peak[i] = d1->peak[i];
      ans->peak[i] = *convolve_single_peak(d1->peak[i], d2->peak[i]);
      //  for (int k=0; k<d1->peak[i].axis.size(); k++) 
      //ans->peak[i].mean[k] = d1->peak[i].mean[k] * d2->peak[i].mean[k];
    }
  }
  else if (d1PeakSize == 1 && d2PeakSize > 1){
    ans = new MoG(name_str,d2PeakSize);
    for (int i=0; i<d2PeakSize; i++){
      //ans->peak[i] = d2->peak[i];
      //d2, d1 order here because 1st argument of convolve_single_peak determines color
      ans->peak[i] = *convolve_single_peak(d2->peak[i], d1->peak[0]);
      //for (int k=0; k<d2->peak[i].axis.size(); k++) 
      //ans->peak[i].mean[k] = d1->peak[0].mean[k] * d2->peak[i].mean[k];
    }

  }
  else if (d1PeakSize > 1 && d2PeakSize == 1){
    ans = new MoG(name_str,d1PeakSize);
    for (int i=0; i<d1PeakSize; i++){
      //ans->peak[i] = d1->peak[i];
      ans->peak[i] = *convolve_single_peak(d1->peak[i], d2->peak[0]);
      //for (int k=0; k<d1->peak[i].axis.size(); k++) 
      //ans->peak[i].mean[k] = d1->peak[i].mean[k] * d2->peak[0].mean[k];
    }

  }
  else{
    cout<<"PWCONV ERROR\n";
    cerr<<"PWCONV ERROR\n";
    exit(0);
  }


    
  cout<<"PWCONV result is:\n";
  ans->print();
  return ((void *)&ans[0]);

  
}


extern "C" void *DIF_mf_overlap(void *arg1, void *arg2){ //dgn reworking to above 4/14
  const double SPHERE = 1000; //avoid_obstacle_sphere (mm) radius
  const int MAX_PEAKS = 5, RNGSize = 500;
  MoG *ans, *dist1, *dist2;
  dist1 = ((MoG *)arg1);
  dist2 = ((MoG *)arg2);
  const int result_size = dist1->peak.size() * dist2->peak.size();
  string name_str = "overlap("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout << "OVERLAP function call" << endl;
  dist1->print();
  dist2->print();
  
  try { ans = new MoG(name_str,result_size); }
  catch (std::bad_alloc& ba) { cerr << "dlibinterface: DIF_mf_overlap; bad_alloc caught: " << ba.what() << endl;}
  //overlap is 'colorblind' aka result/calculation does not worry about RHS colors and just inherits from LHS
  //ans->peak.resize(tempPeakSize);
  for(int i=0;i<dist1->peak.size();i++){
    for(int j=0;j<dist2->peak.size();j++){
      int n = j+i*dist2->peak.size();
      if(dist1->peak[i].freeze ==2){
	// Frozen, do nothing
	cout << "doing nothing\n";
	ans->peak[n] = dist1->peak[i];
	ans->peak[n].freeze = dist1->peak[i].freeze;
	ans->peak[n].peakID = dist1->peak[i].peakID;
      }
      else if (dist1->peak[i].freeze==0){
	// get distance between dist1.peak[i] and dist2->peak[j]
	double distance = sqrt( pow((dist1->peak[i].mean[0]-dist2->peak[j].mean[0]),2)
				+ pow((dist1->peak[i].mean[1]-dist2->peak[j].mean[1]),2) );
	//cerr << "distance = " << distance << endl;
	
	if(distance >= SPHERE){ // SPHERE distance
	  ans->peak[n] = dist1->peak[i];
	  ans->peak[n].weight = 0;
	}
	else{
	  // close enough; RNG and test cases
	  //cerr << "Close enough; evaluating...\n";
	    
	  int joint_size=0;
	  MatrixXd *RNG_A, *RNG_B;
	  try { RNG_A = new MatrixXd(dist1->mvnrng(i,RNGSize)); }
	  catch (std::bad_alloc& ba) { cerr << "bad_alloc caught: " << ba.what() << endl;}
	  
	  try { RNG_B = new MatrixXd(dist2->mvnrng(j,RNGSize)); }
	  catch (std::bad_alloc& ba) { cerr << "bad_alloc caught: " << ba.what() << endl;}
	
	  //MatrixXd RNG_A = dist1->mvnrng(i, RNGSize);
          //MatrixXd RNG_B = dist2->mvnrng(j, RNGSize);
	  MatrixXd jointPts;
	  Vector moving_mean_a;
	  //get percentage of overlap between dist1 and dist2.
	  double bhat = ans->BhatCoefficient (dist1, i, dist2, j);
	  //cerr << "Bhat = " << bhat << endl;

	    
	  //evaluate joint probability with RNGSize points in dist1 and RNGSize points in dist2
	  //this vector is a memory hog.."is there a better way!?"
	  std::vector<p_Owner> jointP; //p_Owner is struct { Vector, double, string } ie: {(x,y), prob, owner<-{"dist1","dist2"}}
	  for (int k=0; k<RNGSize; k++){
	    Vector pt_d1;
	    pt_d1.input('x', (*RNG_A)(k,0));
	    pt_d1.input('y', (*RNG_A)(k,1));
	    double p_d1_at_d1  = dist1->probability(i, pt_d1);
	    double p_d2_at_d1  = dist2->probability(j, pt_d1);
	    double joint_d1    = p_d1_at_d1 * p_d2_at_d1; //factor and clean

	    Vector pt_d2;
	    pt_d2.input('x', (*RNG_B)(k,0));
	    pt_d2.input('y', (*RNG_B)(k,1));
	    double p_d1_at_d2 = dist1->probability(i, pt_d2);
	    double p_d2_at_d2 = dist2->probability(j, pt_d2);
	    double joint_d2   = p_d1_at_d2 * p_d2_at_d2; //factor and clean
	        
	    //make p_Owner objects and push to vector
	    p_Owner pt1, pt2;
	    pt1.v = pt_d1;
	    pt1.p = joint_d1;
	    pt1.owner = "dist1";
	    pt2.v = pt_d2;
	    pt2.p = joint_d2;
	    pt2.owner = "dist2";
	    jointP.push_back(pt1);
	    jointP.push_back(pt2);
	  }//end joint probability calculation/building
	  delete RNG_A, RNG_B;
	  //std::sort(jointP.begin(), jointP.end()); //just use sort in future-dgn
	  std::make_heap(jointP.begin(), jointP.end()); //max-heap
	    
	  double xsum=0,ysum=0, xvar=0, yvar=0, covar=0;
	  //keep bhatpercent of joint dist points(*2 because jointP.size==RNGSize*2
	  int dist1_count=0, num_keep=(bhat) * RNGSize * 2; 
	  for(int k=0;k<num_keep;k++){
	    if (jointP.front().owner == "dist1") 
	      dist1_count++;
	        
	    double x=jointP.front().v.point[0].value[0];
	    double y=jointP.front().v.point[0].value[1];
	        
	    xsum+=x;
	    ysum+=y;
	    pop_heap(jointP.begin(), jointP.end());
	    jointP.pop_back();

	    jointPts.conservativeResize(k+1,2);
	    jointPts(k,0) = x;
	    jointPts(k,1) = y;
	  }
	  jointP.clear(); //destruct internal objects
	  double xmean=xsum/num_keep;
	  if(xsum==0) xmean = 0;
	  double ymean=ysum/num_keep;
	  if(ysum==0) ymean = 0;

	  //this is wrong. MAYBE. also, no 2nd loop needed .. check wikipedia (the subtraction of mean^2 can be factored out)
	  for(int k=0;k<num_keep;k++){
	    xvar+=pow((jointPts(k,0)-xmean),2);
	    yvar+=pow((jointPts(k,1)-ymean),2);
	    covar+=((jointPts(k,0)-xmean)*(jointPts(k,1)-ymean));
	  }
	  int xvarsum = xvar;
	  if(xvar!=0) xvar = xvar/(num_keep-1);
	  if(yvar!=0) yvar = yvar/(num_keep-1);
	  if(covar!=0)covar = covar/(num_keep-1);
	  //cerr << "x=" << xmean << " y=" << ymean << " dist1count= " << dist1_count << endl;
	  //actually set for the current object
	  ans->peak[n].freeze = dist1->peak[i].freeze;
	  ans->peak[n].peakID = dist1->peak[i].peakID;
	  ans->peak[n].weight = (bhat< 0.00001/*1/static_cast<double>(RNGSize)*/ || (xmean == 0 && ymean == 0))? 0 : bhat; 
	  //if just going to set the weight to 0, no need to do all the joint distribution calculation above
	  //just checking if this zero-weighting works first : dgn 3/14
	  //also if avg mean (xmean,ymean) happens to be 0,0, set weight to 0.
	  ans->peak[n].axis.push_back('x');
	  ans-> peak[n].axis.push_back('y');
	  ans->peak[n].mean.conservativeResize(2);
	  ans->peak[n].mean[0] = xmean;
	  ans->peak[n].mean[1] = ymean;
	  ans->peak[n].variance.conservativeResize(2,2);
	  ans->peak[n].variance(0,0) = xvar;
	  ans->peak[n].variance(1,1) = yvar;
	  ans->peak[n].variance(1,0) = covar;
	  ans->peak[n].variance(0,1) = covar;
	}
      }
      else{
	cout << "ERROR in Overlap!! FreezeCase not found.\n";
	exit(1);
      }
    }
  }

  ans->check();
  ans->check_general();
  
  
  cout << "*****Post-overlap checkpoint for " << ans->distribution_name <<"*****\n";
  dist1->print();
  dist2->print();

  
  cout<<"OVERLAP result is:\n";
  if (ans->peak.size()!=0)
    ans->print();
  else cout<<" No overlap, so zero peak MoG has been produced! This is okay in this case.\n";

  //return ((void *)&ans[0]);
  return ((void *)&ans[0]);
  


  }

extern "C" void *DIF_mf_pwdiv(void *arg1, void *arg2){
  MoG *ans,*d1,*d2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  string name_str = "pwdiv("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout<<" PWDIV function call: "<<name_str<<endl;
  int d1PeakSize = d1->peak.size(), d2PeakSize = d2->peak.size();;

  //all cases disjoint, extra case provides commutativity
  //colors and weights and variances inherited from LHS
  //UNLESS one side #peaks == 1 and other > 1 THEN colors and weights and variances inherited from 'other'
  if (d1PeakSize == d2PeakSize){
    ans = new MoG(name_str,d1PeakSize);
    for (int i=0; i<d1PeakSize; i++){
      ans->peak[i] = d1->peak[i];
      for (int k=0; k<d1->peak[i].axis.size(); k++) 
	ans->peak[i].mean[k] = d1->peak[i].mean[k] / d2->peak[i].mean[k];
    }
  }
  else if (d1PeakSize == 1 && d2PeakSize > 1){
    ans = new MoG(name_str,d2PeakSize);
    for (int i=0; i<d2PeakSize; i++){
      //ans->peak[i] = d2->peak[i];
    	ans->peak[i] = d1->peak[0]; //-- dml 10/14
      for (int k=0; k<d2->peak[i].axis.size(); k++) 
	ans->peak[i].mean[k] = d1->peak[0].mean[k] / d2->peak[i].mean[k];
    }

  }
  else if (d1PeakSize > 1 && d2PeakSize == 1){
    ans = new MoG(name_str,d1PeakSize);
    for (int i=0; i<d1PeakSize; i++){
      ans->peak[i] = d1->peak[i];
      for (int k=0; k<d1->peak[i].axis.size(); k++) 
	ans->peak[i].mean[k] = d1->peak[i].mean[k] / d2->peak[0].mean[k];
    }

  }
  else{
    cout<<"PWDIV ERROR\n";
    cerr<<"PWDIV ERROR\n";
    exit(0);
  }

  /*
  int popbacks=0, lastone=ans->peak.size()-1; 
  
  // if the weights are normalized, then OVERLAP/superpose don't work properly, dml 2/14

  //the in place rearrangement of ans (by getting rid of zero-weight peaks) was buggy
  //sacrificing space for clarity -dgn
  int slot=0, valid_count=0;
  for (int i=0; i<ans->peak.size(); i++)
    if (ans->peak[i].weight != 0)
      valid_count++;
  MoG *ans_cleaned = new MoG(name_str,valid_count);
  
  for (vector<distribution>::iterator it = ans->peak.begin(); it != ans->peak.end(); it++)
    if (it->weight != 0){
      ans_cleaned->peak[slot] = *it;
      slot++;}
  
  cout<<"PWDIV result is:\n";
  ans_cleaned->print();
  //return ((void *)&ans[0]);
  return ((void *)&ans_cleaned[0]);
  
  */
    
  cout<<"PWDIV result is:\n";
  ans->print();
  return ((void *)&ans[0]);


}


/*
  color-blind, peak-wise multiplication
  cases:
  1) either LHS or RHS singly-peaked -> multiply single peak by each of the other side
       this logic covers case that BOTH LHS and RHS are singly-peaked.
  2) both LHS and RHS have same #peaks -> for all i in #peaks : LHS.peak[i]  * RHS.peak[i]
  3) otherwise -> error and exit(0)
  -dagan
  TODO : new variance = LHS var + RHS var
 */
extern "C" void *DIF_mf_pwmul(void *arg1, void *arg2){
  MoG *ans,*d1,*d2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  string name_str = "pwmul("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout<<" PWMUL function call: "<<name_str<<endl;
  int d1PeakSize = d1->peak.size(), d2PeakSize = d2->peak.size();;

  //all cases disjoint, extra case provides commutativity
  //colors and weights and variances inherited from LHS
  //UNLESS one side #peaks == 1 and other > 1 THEN colors and weights and variances inherited from 'other'
  if (d1PeakSize == d2PeakSize){
    ans = new MoG(name_str,d1PeakSize);
    for (int i=0; i<d1PeakSize; i++){
      ans->peak[i] = d1->peak[i];
      for (int k=0; k<d1->peak[i].axis.size(); k++) 
	ans->peak[i].mean[k] = d1->peak[i].mean[k] * d2->peak[i].mean[k];
    }
  }
  else if (d1PeakSize == 1 && d2PeakSize > 1){
    ans = new MoG(name_str,d2PeakSize);
    for (int i=0; i<d2PeakSize; i++){
      //ans->peak[i] = d2->peak[i];
      ans->peak[i] = d1->peak[0]; //-- dml 10/14
      for (int k=0; k<d2->peak[i].axis.size(); k++) 
	ans->peak[i].mean[k] = d1->peak[0].mean[k] * d2->peak[i].mean[k];
    }

  }
  else if (d1PeakSize > 1 && d2PeakSize == 1){
    ans = new MoG(name_str,d1PeakSize);
    for (int i=0; i<d1PeakSize; i++){
      ans->peak[i] = d1->peak[i];
      for (int k=0; k<d1->peak[i].axis.size(); k++) 
	ans->peak[i].mean[k] = d1->peak[i].mean[k] * d2->peak[0].mean[k];
    }

  }
  else{
    cout<<"PWMUL ERROR\n";
    cerr<<"PWMUL ERROR\n";
    exit(0);
  }

  /*
  int popbacks=0, lastone=ans->peak.size()-1; 
  
  // if the weights are normalized, then OVERLAP/superpose don't work properly, dml 2/14

  //the in place rearrangement of ans (by getting rid of zero-weight peaks) was buggy
  //sacrificing space for clarity -dgn
  int slot=0, valid_count=0;
  for (int i=0; i<ans->peak.size(); i++)
    if (ans->peak[i].weight != 0)
      valid_count++;
  MoG *ans_cleaned = new MoG(name_str,valid_count);
  
  for (vector<distribution>::iterator it = ans->peak.begin(); it != ans->peak.end(); it++)
    if (it->weight != 0){
      ans_cleaned->peak[slot] = *it;
      slot++;}
  
  cout<<"PWMUL result is:\n";
  ans_cleaned->print();
  //return ((void *)&ans[0]);
  return ((void *)&ans_cleaned[0]);
  
  */
    
  cout<<"PWMUL result is:\n";
  ans->print();
  return ((void *)&ans[0]);


}




extern "C" void *DIF_mf_pwrot(void *arg1, void *arg2){
  MoG *ans,*d1,*d2,*dc2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  string name_str = "pwrot("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";

  cout<<" PWROT function call: "<<name_str<<endl;
  cout<<"d1 peaks: "<<d1->peak.size()<<endl;
  cout<<"d2 peaks: "<<d2->peak.size()<<endl;

  ans = (MoG *)rotateMM(*d1, *d2);

  //  cout<<"d1 peaks: "<<d1->peak.size()<<endl;
  //  cout<<"d2 peaks: "<<d2->peak.size()<<endl;

  ans->print();
  return ((void *)&ans[0]);
}

extern "C" void *DIF_inv_dist(void *arg1){
  MoG *ans,*d1;
  d1 = ((MoG *)arg1);
  string name_str = "invDist("+((MoG*)arg1)->distribution_name+")";
  cout<<" INVDIST function call: "<<name_str<<endl;
  //ans = new MoG(name_str,d1->peak.size());
  //*ans = *d1;
  ans = (MoG *)invDist(d1);
  //ans->distribution_name=name_str;
  ans->print();
  return ((void *)&ans[0]);
}

extern "C" void *DIF_mf_superpose(void *arg1, void *arg2){
  MoG *ans,*d1,*d2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  string name_str = "superpose("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout<<" SUPERPOSE function call: "<<name_str<<endl;
  //ans = new MoG(name_str);
  //*ans = *d1;
  ans = (MoG *)superpose(d1, d2);
  //ans->distribution_name=name_str;
  ans->print();
  return ((void *)&ans[0]);
}

extern "C" void *DIF_repulse(void *arg1, void *arg2, void *arg3){
  MoG *pos = (MoG *)arg1;
  MoG *obs = (MoG *)arg2;
  int *speed = (int*)arg3;
  MoG *ans;
  string name_str = "repulse("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout<<" REPULSE function call: "<<name_str<<endl;
  int obsSize = obs->peak.size();
  
  ans = new MoG(name_str,obsSize);

  vector<distribution>::iterator ans_it = ans->peak.begin();
  for (vector<distribution>::iterator lhs_it = pos->peak.begin(); lhs_it != pos->peak.end(); lhs_it++){
    for (vector<distribution>::iterator rhs_it = obs->peak.begin(); rhs_it != obs->peak.end(); rhs_it++){ 
      if (lhs_it->peakID == rhs_it->peakID){
	*ans_it = *rhs_it; //initialize to obstacle
	ans_it->weight = rhs_it->weight;
	ans_it->peakID = lhs_it->peakID;
	//raw repulsion
	//int axissize = ans_it->axis.size();
	//for (int i=0; i < axissize; i++){
	// ans_it->mean[i] = (*speed)*(lhs_it->mean[i] - rhs_it->mean[i]);
	//}
	       /*//unit()
	double sum=0;
	for (int j=0; j<axissize; j++) {
	  sum += ans_it->mean[j]*ans_it->mean[j];
	}
	double sqrtsum=(sum>0)?(sqrt(sum)):0.0;
	for (int j=0; j<ans_it->axis.size(); j++) {
	  ans_it->mean[j] = (sqrtsum>0)?(ans_it->mean[j]/sqrtsum):0.0;
	}
	//set zero variance here
	
	//scale()
	*/
	
	
	if (ans_it != ans->peak.end()) ans_it++;
      }
      else{
	*ans_it = *lhs_it;
	ans_it->weight = 0;
	if (ans_it != ans->peak.end()) ans_it++;
	
      }
    }
  }
	     
	     
	     
	     




  ans->print();
  return ((void *)&ans[0]);

}

// cap and scale: arg1=mog distance, arg2=real distance threshold, arg3=real to be scaled
// returns a mog with lengths capped to arg3 or arg3 scaled by dist/arg2

// dm; 12/16/2013 I haved added the following two default values below
#define SMALLDIST 0 // stop any oscillation around a goal
#define SMALLSPEED 0.01
#define SATURADIUS 100 //radius of saturation region in mm
#define GOALTOL   (0.0) // account for the fact that we subtract the goal tolerance from the waypoints
                        // change from 500 to 0 for the new cnl

//this version of capscale (older below) implements the idea of a saturation region around the goal -dgn
extern	"C" void *DIF_capscale(void *arg1, void *arg2, void *arg3){
  MoG *ans,*dist = (MoG *)arg1;
  double *threshD= (double*)arg2;
  double *factor = (double*)arg3;

  if (arg1!=0 && arg2!=0 && arg3!=0) {
    printf("DIF Calling capscale(%s,%f,%f)\n",dist->distribution_name.c_str(), *threshD, *factor);
	} else printf("DIF: Calling capscale -- args missing\n");
  
  string name_str = "capscale("+dist->distribution_name+","+ftoa(*threshD)+","+ftoa(*factor)+")";
  ans = new MoG(name_str,dist->peak.size());
  *ans = *dist;
  ans->distribution_name=name_str;
  //else if (fabs (scaled - dist->peak[i].mean[k]) )
  for (int i=0; i<ans->peak.size(); i++)
    for (int k=0; k<ans->peak[i].axis.size(); k++) 
      if (dist->peak[i].mean[k] >= (*threshD)) // dml 5/2015 added =
	ans->peak[i].mean[k] = (float)(*factor);
      else if (dist->peak[i].mean[k] < SATURADIUS){
	ans->peak[i].mean[k] = SMALLSPEED;
	cout<<"dlibinterface: capscale; In saturation radius, issuing small speed\n";
      }
      else {
	ans->peak[i].mean[k] = ((float)*factor) * 
                  ((dist->peak[i].mean[k]+GOALTOL)/((float)*threshD+GOALTOL));//precompute;
      }


  return ((void *)&ans[0]);
}

//depr in favor of saturation version above -dgn
extern	"C" void *DIF_capscale_old(void *arg1, void *arg2, void *arg3){
  MoG *ans,*dist = (MoG *)arg1;
  double *threshD= (double*)arg2;
  double *factor = (double*)arg3;

  if (arg1!=0 && arg2!=0 && arg3!=0) {
    printf("DIF Calling capscale(%s,%f,%f)\n",dist->distribution_name.c_str(), *threshD, *factor);
	} else printf("DIF: Calling capscale -- args missing\n");
  
  string name_str = "capscale("+dist->distribution_name+","+ftoa(*threshD)+","+ftoa(*factor)+")";
  ans = new MoG(name_str,dist->peak.size());
  *ans = *dist;
  ans->distribution_name=name_str;

  for (int i=0; i<ans->peak.size(); i++)
      for (int k=0; k<ans->peak[i].axis.size(); k++) 
	if (dist->peak[i].mean[k] > *threshD) 
	  ans->peak[i].mean[k] = (float)(*factor);
	else if (dist->peak[i].mean[k]<=SMALLDIST)   	 
	  ans->peak[i].mean[k]= SMALLSPEED;
	else {
	  //	  cerr<<"CAPSCALE: dist="<<dist->peak[i].mean[k]<<" thresh="<<*threshD<<" factor="<<*factor<<endl;
	  ans->peak[i].mean[k]=((float)*factor) * ((dist->peak[i].mean[k]+GOALTOL)/((float)*threshD));
	  //      cerr<<"returning "<<ans->peak[i].mean[k]<<endl;
	}

  return ((void *)&ans[0]);
}

// input: arg1 is a vector of lengths from the difference of two unit vectors
//        interpreted as (cos(a),sin(a)) for a rotation of a
//        arg2 is a vector of max allowed lengths (ie max difference of unit vectors)
//        interpreted as (cos(max),sin(max)) for a 1st quadrant rotation of max degrees
// output is either the same as arg1 or is arg1 capped by arg2


double length(Euclidean_vector x) {
  double sum=0;
  for (int i=0; i<x.axis.size(); i++) sum += x.value(i)*x.value(i);
  if (sum>0) return sqrt(sum); else return 0;
}

extern "C" void *DIF_capvector(void *arg1, void *arg2) {
  Vector *vecIn    = (Vector *)arg1;
  Vector *maxV     = (Vector *)arg2;

  string name="capvector("+vecIn->name+","+maxV->name+")";
  Vector *ans = new Vector(name);
  *ans = *vecIn;
  ans->name = name;

  for (int i=0; i<vecIn->point.size(); i++) {

    // avoid 180 degree turn issues, does not work very well
    if (vecIn->point[i].value(0)==0) vecIn->point[i].value(0)=0.1; 
    if (vecIn->point[i].value(1)==0) vecIn->point[i].value(1)=0.1; 

    double normV = length(vecIn->point[i]), normM = length(maxV->point[i]);

    //cout << "DIF: capvector;  i=" << i << " |vecIn|=" << normV
    //   <<" |maxV|="<<normM<<" maxV(x,y)=("<<maxV->point[i].value(0)<<","<<maxV->point[i].value(1)<<")"<<endl;

    if ( normV > maxV->point[i].value(1) ) {
        for (int j=0; j<vecIn->point[i].axis.size(); j++) 
	  ans->point[i].value(j) = maxV->point[i].value(1)*(vecIn->point[i].value(j)/normV); 
	  //    	  ans->point[i].value(j) = SIGN(vecIn->point[i].value(j)) * maxV->point[i].value(j) ; 
    }
  }

  cout<<"DIF: capvector; input[0]  is "<<vecIn->point[0].value<<endl ;
  cout<<"DIF: capvector; max[0]  is "  <<maxV->point[0].value<<endl ;
  cout<<"DIF: capvector; output[0] is "<<ans->point[0].value<<endl ;
  return ans;  
}

extern "C" void *DIF_rotation(void *arg1, void *arg2) {
  Vector *vecIn    = (Vector *)arg1;
  Vector *maxV     = (Vector *)arg2;

  string name="rotation("+vecIn->name+","+maxV->name+")";
  Vector *ans = new Vector(name);
  *ans = *vecIn;
  ans->name = name;

  for (int i=0; i<vecIn->point.size(); i++) {
     double theta=maxV->point[i].value(1);
     ans->point[i].value(0) = cos(theta)*vecIn->point[i].value(0) + (-1)*sin(theta)*vecIn->point[i].value(1) - vecIn->point[i].value(0);
     ans->point[i].value(1) = sin(theta)*vecIn->point[i].value(0) + cos(theta)*vecIn->point[i].value(1) - vecIn->point[i].value(1);
    }
    
  cout<<"DIF: rotation; input[0]  is "<<vecIn->point[0].value<<endl ;
  cout<<"DIF: rotation; max[0]  is "  <<maxV->point[0].value<<endl ;
  cout<<"DIF: rotation; output[0] is "<<ans->point[0].value<<endl ;
  return ans;

  cout<<"function rotation is called"<<endl;
  return ans;
}

extern "C" void *DIF_smaller(void *arg1, void *arg2) {
  Vector *vecIn    = (Vector *)arg1;
  Vector *maxV     = (Vector *)arg2;

  string name="truncate("+vecIn->name+","+maxV->name+")";
  Vector *ans = new Vector(name);
  
  *ans = *maxV;
  ans->name = name;

  for (int i=0; i<vecIn->point.size(); i++) {
  	if (vecIn->point[i].value(0)>0)	// When the turning direction is anti-clockwise
	    if ( vecIn->point[i].value(0) < maxV->point[i].value(0) ) 
	    {
	        for (int j=0; j<vecIn->point[i].axis.size(); j++) 
		  		ans->point[i].value(j) = vecIn->point[i].value(j); 
	    }
	    else
	    {
	        for (int j=0; j<maxV->point[i].axis.size(); j++) 
		  		ans->point[i].value(j) = maxV->point[i].value(j); 
	    }
	else	// When the turning direction is clockwise
	    if ( vecIn->point[i].value(0) < -1 * maxV->point[i].value(0) ) 
	    {
	        for (int j=0; j<maxV->point[i].axis.size(); j++) 
		  		ans->point[i].value(j) = -1 * maxV->point[i].value(j); 
	    }
	    else
	    {
	        for (int j=0; j<vecIn->point[i].axis.size(); j++) 
		  		ans->point[i].value(j) = vecIn->point[i].value(j); 
	    }
  }

  cout<<"DIF: truncate; input[0]  is "<<vecIn->point[0].value<<endl ;
  cout<<"DIF: truncate; max[0]  is "  <<maxV->point[0].value<<endl ;
  cout<<"DIF: truncate; output[0] is "<<ans->point[0].value<<endl ;
  return ans;  
}

extern "C" void *DIF_vector(void *arg){
  MoG *inmog = (MoG *)arg;
  string name_str = "vector("+inmog->distribution_name+")";
  Vector *outvec = new Vector(name_str);
  outvec->point.resize(0);
  outvec->name=name_str;

  for (int i=0; i<inmog->peak.size(); i++) {
    Euclidean_vector vec;
    cout<<"DIF_vector: peak"<<i;
    vec.axis = inmog->peak[i].axis;
    //cout<<" axis ";
    for (int j=0; j<vec.axis.size(); j++) cout<<vec.axis[j]<<" ";
    vec.value=inmog->peak[i].mean;
    //cout<<" value "<<vec.value<<endl;
    outvec->point.push_back(vec);
  }
  //cout <<"DIF_vector: size of point vector: "<<outvec->point.size()<<endl;
  cout<<"DIF_vector IN:"<<name_str<<"="<<inmog->peak[0].mean(0)<<","<<inmog->peak[0].mean(1)<<endl;
  cout<<"DIF_vector OUT:"<<name_str<<"="<<outvec->point[0].value(0)<<","<<outvec->point[0].value(1)<<endl;
  //cout<<"DIF_vector:"<<name_str<<" output="; outvec->print(); cout<<endl;

  return (void *)outvec;
}

// caps the value of all means of all peaks of arg1 between arg2(high)..arg3(low)

extern "C" void *DIF_cap(void *arg1, void *arg2, void *arg3) {
  MoG *ans,*dist = (MoG *)arg1;
  double *threshD= (double*)arg2;
  double *threshDNeg= (double*)arg3;

  if (arg1!=0 && arg2!=0 && arg3!=0) {
    printf("DIF: Calling cap(%s,%f,%f)\n",dist->distribution_name.c_str(), *threshD, *threshDNeg);
  } 
  else {
    printf("DIF: Calling cap-- some args missing\n");
    exit(0);
  }
  
  string name_str = "cap("+dist->distribution_name+","+ftoa(*threshD)+","+ftoa(*threshDNeg)+")";
  ans = new MoG(name_str,dist->peak.size());
  *ans = *dist;
  ans->distribution_name=name_str;

  for (int i=0; i<ans->peak.size(); i++) {
    double sum=0;
    for (int k=0; k<ans->peak[i].axis.size(); k++)  sum  += ans->peak[i].mean[k]*ans->peak[i].mean[k];
    double sqrtsum = (sum>0)?sqrt(sum):0.0;
    if (sqrtsum > *threshD) {
      for (int k=0; k<ans->peak[i].axis.size(); k++)  ans->peak[i].mean[k] *= (*threshD)/sqrtsum;
      printf("DIF: Cap cap applied (%f>%f).\n",sqrtsum,*threshD);
    } 
  }

  return ((void *)&ans[0]);

}

//**************************
// Array Routines
//**************************

vector<array *> DIF_agarbage_list; // for garbage collection!

extern "C" void *DIF_agarbage_cleanup(void *notme) {
  for (int i=0; i<DIF_agarbage_list.size(); i++)
    if (DIF_agarbage_list[i]!=(array *)notme) delete DIF_agarbage_list[i];
  DIF_agarbage_list.clear();
}

extern "C" void *DIF_ascale(void *arg1, void *arg2) {
  array   *arr    = (array *) arg1;
  double  *scale  = (double *)arg2;

  stringstream s;
  s<<*scale;
  string name="ascale("+arr->name+","+s.str()+")";
  array *ans = new array;
  DIF_agarbage_list.push_back(ans);

  *ans=*arr; // copy all the flags etc from arg1

  ans->name=name;  
  if (arr->arraytype==ARRAYREAL) {
    // overwrite datptr and calculate result
    ans->arrayReal = new vector<float>;
    ans->arrayReal->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    for (int i=0; i<arr->dimx; i++)  (*ans->arrayReal)[i]=(*arr->arrayReal)[i] * (*scale);

  } else if (arr->arraytype==ARRAYMOG) {
    ans->arrayMog = new vector<dist>;
    ans->arrayMog->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    
    for (int i=0; i<ans->dimx; i++) {
      dist temp = (*arr->arrayMog)[i];
      for (int j=0; j<temp.value.peak.size(); j++) 
	temp.value.peak[j].mean[0] *= (*scale), temp.value.peak[j].mean[1] *= (*scale);
      (*ans->arrayMog)[i]=temp;
    }
  } else if (arr->arraytype==ARRAY2MOG) {
    ans->array2Mog = new vector< vector<float> >;
    ans->array2Mog->resize(ans->dimx);
    for (int i=0; i<ans->dimx; i++) (*ans->array2Mog)[i].resize(ans->dimy);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    for (int i=0; i<arr->dimx; i++)  
      for (int j=0; j<arr->dimy; j++) ((*ans->array2Mog)[i])[j]=((*arr->array2Mog)[i])[j] * (*scale);
  }
  ans->print();
  return ans;
}//mf_ascale


extern "C" void DIF_asumop(void *arg1, void *arg2, char op1, char op2, void **res) {
  array   *arr    = (array *) arg1;
  double  *offset  = (double *)arg2;
  stringstream s;
  s<<*offset;

  int sign1 = (op1=='+')?1:-1,
    sign2 = (op2=='+')?1:-1;

  string name="asum("+arr->name+","+s.str()+")";
  array *ans = new array;
  DIF_agarbage_list.push_back(ans);
  *ans=*arr; // copy all the flags etc from arg1

  
  ans->name=name;  
  if (arr->arraytype==ARRAYREAL) {
    // overwrite datptr and calculate result
    ans->arrayReal = new vector<float>;
    ans->arrayReal->resize(arr->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    for (int i=0; i<arr->dimx; i++)  (*ans->arrayReal)[i] = sign1*(*arr->arrayReal)[i]
				                           +sign2*(*offset);

  } else if (arr->arraytype==ARRAYMOG) {
    ans->arrayMog = new vector<dist>;
    ans->arrayMog->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    
    for (int i=0; i<ans->dimx; i++) {
      dist temp = (*arr->arrayMog)[i];
      for (int j=0; j<temp.value.peak.size(); j++) 
	temp.value.peak[j].mean[0] = sign1*temp.value.peak[j].mean[0] + sign2*(*offset), 
	  temp.value.peak[j].mean[1] = sign1*temp.value.peak[j].mean[1] + sign2*(*offset);
      (*ans->arrayMog)[i]=temp;
    }
  } else if (arr->arraytype==ARRAY2MOG) {
    ans->array2Mog = new vector< vector<float> >;
    ans->array2Mog->resize(ans->dimx);
    for (int i=0; i<ans->dimx; i++) (*ans->array2Mog)[i].resize(ans->dimy);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    for (int i=0; i<arr->dimx; i++)  
      for (int j=0; j<arr->dimy; j++) 
	((*ans->array2Mog)[i])[j]=sign1*((*arr->array2Mog)[i])[j] + sign2*(*offset);
  }
  ans->print();
  (*res) = ans;

  return;// ans;
}//mf_asum

extern "C" void *DIF_asum(void *arg1, void *arg2) {
  int *res;
  DIF_asumop(arg1, arg2, '+','+',(void **)&res);
  return (void *)res;
}

extern "C" void *DIF_adiff(void *arg1, void *arg2) {
  int *res;
  DIF_asumop(arg1, arg2, '+','-',(void **)&res);
  return (void *)res;
}



extern "C" void DIF_asumdop(void *arg1, void *arg2, char op1, char op2, void ** res) {
  array   *arr    = (array *) arg1;
  MoG  *d1  = (MoG *)arg2;

  int sign1 = (op1=='+')?1:-1,
    sign2 = (op2=='+')?1:-1;

  string name="asumd("+arr->name+","+d1->distribution_name+")";
  array *ans = new array;
  DIF_agarbage_list.push_back(ans);
  *ans=*arr; // copy all the flags etc from arg1

  
  ans->name=name;  
  if (arr->arraytype==ARRAYREAL) {
    // overwrite datptr and calculate result

    cout<<"asumd not defined for real arrays\n";
    exit(0);
  } else if (arr->arraytype==ARRAYMOG) {
    ans->arrayMog = new vector<dist>;
    ans->arrayMog->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    
    for (int i=0; i<ans->dimx; i++) {
      dist temp = (*arr->arrayMog)[i];
      for (int j=0; j<temp.value.peak.size(); j++) {
	temp.value.peak[j].mean[0] = sign1*temp.value.peak[j].mean[0] + 
	  sign2*d1->peak[j].mean[0];
	temp.value.peak[j].mean[1] = sign1*temp.value.peak[j].mean[1] + 
	  sign2*d1->peak[j].mean[1];
      }
      (*ans->arrayMog)[i]=temp;
    }
  } else if (arr->arraytype==ARRAY2MOG) {

    cout<<"asumd not defined for real arrays\n";
    exit(0);
  }

  ans->print();
  (*res) = (void *)ans;

  return;// ans;
}//mf_asumd



extern "C" void *DIF_apwmul(void *arg1, void *arg2) {
  array   *arr    = (array *) arg1;
  array   *arr2    = (array *) arg2;


  string name="apwmul("+arr->name+","+arr2->name+")";
  array *ans = new array;
  DIF_agarbage_list.push_back(ans);

  *ans=*arr; // copy all the flags etc from arg1

  ans->name=name;  
  if (arr->arraytype==ARRAYREAL||arr2->arraytype==ARRAYREAL) {
    // overwrite datptr and calculate result
    ans->arrayReal = new vector<float>;
    ans->arrayReal->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    cout<<"APWMUL: Not defined for REAL array\n";
    exit(0);

  } else if (arr->arraytype==ARRAYMOG && arr2->arraytype==ARRAYMOG) {
    ans->dimx=min(ans->dimx,arr2->dimx); 
    ans->arrayMog = new vector<dist>;
    ans->arrayMog->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    for (int i=0; i<ans->dimx; i++) {
      dist temp1 = (*arr->arrayMog)[i];
      dist temp2 = (*arr2->arrayMog)[i];

      MoG *resMog = (MoG *)DIF_mf_pwmul((void *)&temp1.value, (void *)&temp2.value);
      temp1.value = *resMog;
      (*ans->arrayMog)[i]=temp1;
      delete resMog; // careful once we add distribution garbage collection
    }
  } else if (arr->arraytype==ARRAY2MOG||arr2->arraytype==ARRAY2MOG) {
    ans->array2Mog = new vector< vector<float> >;
    ans->array2Mog->resize(ans->dimx);
    for (int i=0; i<ans->dimx; i++) (*ans->array2Mog)[i].resize(ans->dimy);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    cout<<"APWMUL: Not defined for ARRAY2MOG in "<<name<<endl;
    exit(0);
  } else {
    cout<<"APWMUL: incompatible array types in "<<name<<endl;
    exit(0);
  }
  ans->print();
  return ans;
}//mf_apwmul


extern "C" void *DIF_acap(void *arg1, void *arg2) {
  array   *arr    = (array *) arg1;
  cout<<"acap NOT DEFINED YET.\n"; exit(0);
}//mf_acap


extern "C" void *DIF_arot(void *arg1, void *arg2) {
  array   *arr    = (array *) arg1;
  MoG     *angOffset  = (MoG *)arg2;

  string name="arot("+arr->name+","+angOffset->distribution_name+")";
  array *ans = new array;
  DIF_agarbage_list.push_back(ans);
  cout<<name<<":: ";

  *ans=*arr; // copy all the flags etc from arg1

  ans->name=name;  
  if (arr->arraytype==ARRAYREAL) {
    // overwrite datptr and calculate result
    ans->arrayReal = new vector<float>;
    ans->arrayReal->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    cout<<"arot NOT DEFINED YET.\n";
    exit(0);

  } else if (arr->arraytype==ARRAYMOG) {
    ans->arrayMog = new vector<dist>;
    ans->arrayMog->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    
    for (int i=0; i<ans->dimx; i++) {
      dist temp = (*arr->arrayMog)[i];
      if (temp.value.peak.size()!=angOffset->peak.size()) {
	cout<<"AROT: ERROR; diff num peaks in both arguments "<<name<<endl;
	exit(0);
      }
      for (int j=0; j<angOffset->peak.size(); j++) {// colors
	double cAng = atan2( angOffset->peak[j].mean[1], angOffset->peak[j].mean[0]);
        double oAng = atan2(temp.value.peak[j].mean[1], temp.value.peak[j].mean[0]);
	temp.value.peak[j].mean[0]= cos(cAng+oAng);
	temp.value.peak[j].mean[1]= sin(cAng+oAng);
	double rAng = atan2(temp.value.peak[j].mean[1], temp.value.peak[j].mean[0]);
	cout<<i<<": orig Ang="<<180*oAng/M_PI<<" del Ang="<<180*cAng/M_PI<<"->"<<180*rAng/M_PI<<", ";
      }
      cout<<endl;
      (*ans->arrayMog)[i]=temp;
    }
  } else if (arr->arraytype==ARRAY2MOG) {
    ans->array2Mog = new vector< vector<float> >;
    ans->array2Mog->resize(ans->dimx);
    for (int i=0; i<ans->dimx; i++) (*ans->array2Mog)[i].resize(ans->dimy);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    cout<<"arot NOT DEFINED YET.\n";
    exit(0);

  }
  ans->print();
  return ans;
}//mf_rot


extern "C" void *DIF_mf_acut(void *arg1, void *arg2) {
  array   *arr    = (array *) arg1;
  double ct = *(double *)arg2;
  stringstream ss;
  ss<<ct;
  string name="acut("+arr->name+","+ss.str()+")";
  array *ans = new array;
  DIF_agarbage_list.push_back(ans);
  *ans=*arr; // copy all the flags etc from arg1
  
  ans->name=name;  
  if (arr->arraytype==ARRAYREAL) {
    // overwrite datptr and calculate result
    ans->arrayReal = new vector<float>;
    ans->arrayReal->resize(arr->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    for (int i=0; i<arr->dimx; i++)  
      if ((*arr->arrayReal)[i]>=ct) (*ans->arrayReal)[i] = ct ;
      else (*ans->arrayReal)[i] = (*arr->arrayReal)[i];

  } else if (arr->arraytype==ARRAYMOG) {
    ans->arrayMog = new vector<dist>;
    ans->arrayMog->resize(ans->dimx);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete
    
    for (int i=0; i<ans->dimx; i++) {
      dist temp = (*arr->arrayMog)[i];
      for (int j=0; j<temp.value.peak.size(); j++) {
	if (temp.value.peak[j].mean[0] >=ct) temp.value.peak[j].mean[0]=ct; 
	if (temp.value.peak[j].mean[1] >=ct) temp.value.peak[j].mean[1]=ct;
      }
      (*ans->arrayMog)[i]=temp;
    }
  } else if (arr->arraytype==ARRAY2MOG) {
    ans->array2Mog = new vector< vector<float> >;
    ans->array2Mog->resize(ans->dimx);
    for (int i=0; i<ans->dimx; i++) (*ans->array2Mog)[i].resize(ans->dimy);
    ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

    for (int i=0; i<arr->dimx; i++)  
      for (int j=0; j<arr->dimy; j++) 
	if (((*arr->array2Mog)[i])[j]>=ct) ((*ans->array2Mog)[i])[j] = ct ;
	  else ((*ans->array2Mog)[i])[j] = ((*arr->array2Mog)[i])[j];
  }
  ans->print();
  
  return ans;
} //mf_acut


extern "C" void *DIF_mf_areduce(void *arg1) {
  array   *arr    = (array *) arg1;
  MoG *ans;
  string name = "areduce("+arr->name+")";

  if (arr->arraytype==ARRAYREAL) {
    cout<<"areduce not defined for this type.\n";
    exit(0);

  } else if (arr->arraytype==ARRAYMOG) {

    dist tempSum = (*arr->arrayMog)[0];
    ans = new MoG(name,tempSum.value.peak.size());
    *ans = tempSum.value;
    ans->distribution_name=name;
    
    int nzeroCount=0;
    for (int i=1; i<arr->dimx; i++) {
      dist temp = (*arr->arrayMog)[i];
      for (int j=0; j<temp.value.peak.size(); j++) {
	if (abs(temp.value.peak[j].mean[0])>0.01 || abs(temp.value.peak[j].mean[1])>0.01) nzeroCount++;
	tempSum.value.peak[j].mean[0] += temp.value.peak[j].mean[0] ;
	tempSum.value.peak[j].mean[1] += temp.value.peak[j].mean[1] ;
	for (int k=0; k<4; k++) 
	  tempSum.value.peak[j].variance(k) += temp.value.peak[j].variance(k);	    
      }// this peak/member of this element
    }// this array element
#if 1
    { float gain=3.3;
      cout<<"AREDUCE: sum (of "<<nzeroCount<<") with g="<<gain<<endl;
      tempSum.value.print();
      if (nzeroCount>0) // modified 5/15 dml, only count nonzero obstacles
	for (int j=0; j<tempSum.value.peak.size(); j++) {
	  for (int k=0; k<2; k++) 
	    tempSum.value.peak[j].mean[k] /= nzeroCount/gain;
	  for (int k=0; k<4; k++) 
	    tempSum.value.peak[j].variance(k) /= (nzeroCount/(gain*gain)); //arr->dimx;
	}// all entries, all peaks averaged.
    }
#endif
	
    *ans = tempSum.value;
    ans->distribution_name=name;

  } else if (arr->arraytype==ARRAY2MOG) {
    cout<<"areduce not defined for this type.\n";
    exit(0);
  }
  ans->print();
  
  return ans;
} 


// Calculate the laser returns for the robot at pos arg2 using map arg1
// by using a scan line algorithm
//
// arguments: 2D array map, distrb robot location, distrib robot heading, 
//  real num lasers, real laser inc
// However, last two arguments are fixed to constants for this model robot for now
//
extern "C" void *DIF_ascanline(void *arg1/*map*/, void *arg2/*pos*/, void *arg3/*heading*/, void *arg4/* laser angles*/) {
  array   *arr    = (array *) arg1;
  MoG     *pose  = (MoG *)arg2;
  MoG     *head  = (MoG *)arg3;
  array   *ang    = (array *) arg4;

  string name="ascanline("+arr->name+","+pose->distribution_name+","
                         +head->distribution_name+","+ang->name+")";
  array *ans = new array;
  DIF_agarbage_list.push_back(ans);

  ans->name=name;  
  ans->arraytype=ARRAYMOG;
  ans->dimx = ang->dimx; ans->dimy=0; // number of laser returns is size of array
  cout<<"SCANLINE: laser config, num lasers="<<ang->dimx<<endl;

  ans->arrayMog = new vector<dist>;
  ans->arrayMog->resize(ang->dimx);

  for (int k=0; k<ang->dimx; k++) (*ans->arrayMog)[k]=(*ang->arrayMog)[k];

  ans->deleteFlag=true; // mark data ptr to be cleaned up on delete

  for (int i=0; i<pose->peak.size(); i++) { // for every color of position and heading
    float base_x= pose->peak[i].mean[0];
    float base_y= pose->peak[i].mean[1];
    float covar_xx= pose->peak[i].variance(0,0);
    float covar_yy= pose->peak[i].variance(1,1);
    float head_cx = head->peak[i].mean[0];
    float head_sx = head->peak[i].mean[1];
    double heading_angle = atan2(double(head_sx), double(head_cx));

    cout<<"SCANLINE: Map size is "<<arr->dimx<<" X "<<arr->dimy<<endl;
    cout<<"SCANLINE: base_x="<<base_x<<" base_y="<<base_y<<endl;

    if (base_x<0 || base_y<0) {
      cout<<"SCANLINE: Robot in negative gird index!\n";
      exit(0);
    }

    if (base_x>arr->dimx || base_y>arr->dimy) {
      cout<<"SCANLINE: Robot is outside the map index range\n";
      exit(0);
    }
   
    if ( (*arr->array2Mog)[int(base_y)][int(base_x)]==1 ||(*arr->array2Mog)[int(base_y)][int(base_x)]==-1 ){
      cout<<"SCANLINE: robot ("<<base_x<<","<<base_y<<") on occupied/unexplored cell of map!\n";
      cerr<<"SCANLINE: robot ("<<base_x<<","<<base_y<<") on occupied/unexplored cell of map!\n";
      exit(0);
    }

    // cout<<"SCANLINE: head m[0]="<<head->peak[i].mean[0]<<" m[1]="<<head->peak[i].mean[1]<<endl;
    cout<<"SCANLINE: Heading angle="<<heading_angle<<endl;
#if 0 // print a map with the position on it; same name as map array variable suffixed with position
    arr->printMapFile(base_x,base_y);
#endif
    //float scan_angle_inc = laser_inc; 
    // add this on to get scan_angle for every additional laser return

    if ((base_x>=0&&base_x<arr->dimx) &&(base_y>=0&&base_y<arr->dimy) ){//on map
      for (int j=0; j<ans->dimx; j++) { // each scan line
        float s_x=base_x, s_y=base_y;
        float distance=0;
	bool intersectFlag=false;
	dist tempScan = (*ang->arrayMog)[j];
	float scanAng = atan2( tempScan.value.peak[0].mean[1], tempScan.value.peak[0].mean[0]);

	cout<<"SCANLINE:c"<<i<<"("<<base_x<<","<<base_y<<","
	    <<heading_angle*180/M_PI<<")@s"
	    <<180*(scanAng/M_PI);

	do  // ray casting
	  if ( (*arr->array2Mog)[int(s_y)][int(s_x)]==1 ||(*arr->array2Mog)[int(s_y)][int(s_x)]==-1 ){
	    dist temp = (*ans->arrayMog)[j];
	    temp.value.input('x',distance,
			     sqrt(pow(covar_xx*cos(heading_angle-scanAng),2) 
				  +pow(covar_yy*sin(heading_angle-scanAng),2) ),
			     i/*colorpeak*/);
	    temp.value.input('y',distance,
			     sqrt(pow(covar_xx*cos(heading_angle-scanAng),2) 
				  +pow(covar_yy*sin(heading_angle-scanAng),2) ),
			     i/*colorpeak*/);
            // has to be same X and Y so it can be used as a scale
	    intersectFlag=true;
	    (*ans->arrayMog)[j]=temp;
	    cout<<"*@d="<<distance<<endl;
	  }
	  else {
	    s_x += cos(heading_angle+scanAng);
	    s_y += sin(heading_angle+scanAng);
	    distance = sqrt( pow(base_x-s_x,2)+pow(base_y-s_y,2));
	    // cout<<"("<<int(s_x)<<","<<int(s_y)<<")";
	  } 
        while (!intersectFlag && s_x>=0 && s_x<arr->dimx && s_y >=0 && s_y <arr->dimy);

	//scan_angle -= scan_angle_inc; // next scan line
      }
    } else {
      dist temp;
        cout<<"SCANLINE: Robot not on map\n";
        cerr<<"SCANLINE: Robot not on map\n";
	ans->dimx=1;
	ans->arrayMog->resize(ang->dimx);
	temp.value.input('x',0,0,0);
	temp.value.input('y',0,0,0);
	(*ans->arrayMog)[0]=temp;
	exit(0);
       }
   }
  cout<<"SCANLINE: result is\n";
  ans->print();
  return ans;
}



extern "C" void *DIF_mf_rosamcl(void *arg1, void *arg2){
  MoG *ans,*d1,*d2;
  d1 = ((MoG *)arg1);
  d2 = ((MoG *)arg2);
  string name_str = "rosamcl("+((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+")";
  cout<<" ROS AMCL : "<<name_str<<endl;
  ans = new MoG(name_str);
  *ans = *d1;

  /* Instead of just setting ans equal to whatever d1 was,
     The actual ros procedure should be called now
  */
  //AMCL a("/home/pengtang/catkin_ws/src/stdr_simulator/stdr_resources/maps/map.yaml");
  //extern AMCL a;
  AMCL a;
  vector<double> amcl_decipher;
  double distance;

  cerr<<"The d1 (mean) is "<< d1->peak[0].mean[0] <<"  "<< d1->peak[0].mean[1]<<endl; 
  cerr<<"The d2 (mean) is "<< d2->peak[0].mean[0] <<"  "<< d2->peak[0].mean[1]<<endl;

  // Cheat, for now
  if (d1->peak[0].mean[0] == 0)
  {
  	ans->peak[0].mean[0] = 2000;
  	ans->peak[0].mean[1] = -2000;
  	ans->peak[0].variance(0,0) = 100000;
  	ans->peak[0].variance(0,1) = 0;
  	ans->peak[0].variance(1,0) = 0;
  	ans->peak[0].variance(1,1) = 100000;
  	return ans;
  }

  distance = sqrt(pow(d1->peak[0].mean[0]/1000, 2) +  pow(d2->peak[0].mean[0]/1000, 2));
  cerr<<" The distance is "<< distance<<endl;
  amcl_decipher = a.AMCL_run(distance, 0);  

  ans->peak[0].mean[0] = amcl_decipher[0] * 1000;
  ans->peak[0].mean[1] = -1 * amcl_decipher[1] * 1000;


  ans->peak[0].variance(0,0) = pow(sqrt(abs(amcl_decipher[4])) * 1000, 2) ;
  ans->peak[0].variance(0,1) = pow(sqrt(abs(amcl_decipher[5])) * 1000, 2) ;
  ans->peak[0].variance(1,0) = pow(sqrt(abs(amcl_decipher[6])) * 1000, 2) ;
  ans->peak[0].variance(1,1) = pow(sqrt(abs(amcl_decipher[7])) * 1000, 2) ;

  return ans; //((void *)&ans[0]);
}

extern "C" void *DIF_mf_rosamcl2(void *arg1, void *arg2, void *arg3, void *arg4){
	MoG *ans,*d1,*d2,*d3,*d4;
	d1 = ((MoG *)arg1);
	d2 = ((MoG *)arg2);
	d3 = ((MoG *)arg3);
	d4 = ((MoG *)arg4);
	double PI = 3.14159;
	string name_str = "rosamcl2("+ ((MoG*)arg1)->distribution_name+","+((MoG*)arg2)->distribution_name+","
								   + ((MoG*)arg3)->distribution_name+","+((MoG*)arg4)->distribution_name+")";

	cout<<" ROS AMCL2 : "<<name_str<<endl;
	ans = new MoG(name_str);
	*ans = *d2;

	if (d1->peak.size()>1 || d1->peak.size()>1 || d1->peak.size()>1 || d1->peak.size()>1 ) {
		cout<<"ROSAMCL ERROR; argument has more than one peak\n";
		cerr<<"ROSAMCL ERROR; argument has more than one peak\n"<<endl;
		exit(0);
	}

	AMCL a;
	vector<double> amcl_decipher;

	// yforward needs to multiply by -1, since in pars it is multiplied by -1
	double xforward = d2->peak[0].mean(0) - d1->peak[0].mean(0);
	double yforward = -1 * (d2->peak[0].mean(1) - d1->peak[0].mean(1));
 
	cout<<"Pos done"<<endl;

	double desired_angle = atan2(-1 * d3->peak[0].mean(1) , d3->peak[0].mean(0)); // y,x
	double previous_angle = atan2(-1 * d4->peak[0].mean(1) , d4->peak[0].mean(0)); // y,x
	double angle = desired_angle - previous_angle;

	cerr<<"Previous pos is: x: "<< d1->peak[0].mean(0)<<" y: "<<d1->peak[0].mean(1)<<endl;
	cerr<<"Desired pos is: x: "<< d2->peak[0].mean(0)<<" y: "<<d2->peak[0].mean(1)<<endl;
	cout<<"ROSAMCL2("<<xforward<<","<<yforward<<","<<angle<<")\n";

  // Cheat, for now
  if (xforward == 0 && yforward == 0)
  {
  	return ans;
  }
  
  double my_forward = sqrt(xforward*xforward + yforward*yforward);
  double angle_needs_spinto = atan2(yforward, xforward);

  // Make sure angle_needs_spinto - old_angle is in [-pi ~ pi)
   if (angle_needs_spinto - previous_angle - desired_angle > PI)
   	  while (angle_needs_spinto - previous_angle - desired_angle > PI)
   	  	  angle_needs_spinto -= 2 * PI;
   else if (angle_needs_spinto - previous_angle - desired_angle < -1 * PI)
   	  while (angle_needs_spinto - previous_angle - desired_angle < -1 * PI)
   	      angle_needs_spinto += 2 * PI;

  std::cerr << "dlibinterface distance x: " << xforward << std::endl;
  std::cerr << "dlibinterface distance y: " << yforward << std::endl;
  std::cerr << "dlibinterface distance: " << my_forward << std::endl;
  std::cerr << "angle_needs_spinto is " << angle_needs_spinto << " previous_angle is  "<<previous_angle<<  "  desired_angle is " << desired_angle<< "  spin angel: " << angle_needs_spinto - previous_angle -desired_angle << std::endl;
  
  bool need_spin_again = false;
  if (std::abs(angle_needs_spinto - desired_angle) > 0.15)
  {
  	std::cerr<<"I will spin this time"<<std::endl;
  	amcl_decipher = a.AMCL_run(0.0, angle_needs_spinto - desired_angle);
  	need_spin_again = true;
  }
  else
  	std::cerr<<"I will not spin this time"<<std::endl;

  amcl_decipher = a.AMCL_run(my_forward / 1000, 0.0);
  
  if (need_spin_again)
  {
  	std::cerr<<"I'm spinning back to the original angle"<<std::endl;
  	amcl_decipher = a.AMCL_run(0.0, desired_angle - angle_needs_spinto);
  }

  ans->peak[0].mean[0] = amcl_decipher[0] * 1000;
  ans->peak[0].mean[1] = -1 * amcl_decipher[1] * 1000;

  ans->peak[0].variance(0,0) = pow(sqrt(abs(amcl_decipher[4])) * 1000, 2) ;
  ans->peak[0].variance(0,1) = pow(sqrt(abs(amcl_decipher[5])) * 1000, 2) ;
  ans->peak[0].variance(1,0) = pow(sqrt(abs(amcl_decipher[6])) * 1000, 2) ;
  ans->peak[0].variance(1,1) = pow(sqrt(abs(amcl_decipher[7])) * 1000, 2) ;


  /* Instead of just setting ans equal to whatever d1 was,
     The actual ros procedure should be called now
     with parameters xforward, yforward and angle
  */

  // the mean of ans should be set to the returned localized position
  // the variance of ans should be set to the returned variance

  return ans; //((void *)&ans[0]);
}
