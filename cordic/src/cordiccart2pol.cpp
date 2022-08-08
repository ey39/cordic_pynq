#include "cordiccart2pol.h"

//角度查找表
ap_fixed<W, I, AP_RND, AP_WRAP, 1> cordic_phase[NO_ITER] = {
    45, 26.56505118, 14.03624347, 7.125016349,
    3.576334375, 1.789910608, 0.8951737102, 0.4476141709,
    0.2238105004, 0.1119056771, 0.05595289189, 0.02797645262,
    0.01398822714,0.006994113675, 0.003497056851, 0.001748528427  //...
};

void cordiccart2pol(data_t x, data_t y, data_t *r, data_t *theta){
#pragma HLS INTERFACE s_axilite port=x  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=y  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=r  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=theta  bundle=CTRL
//return端口很重要，定义后才能在block_design连接上ap_ctrl，与zynq的axilite才能握手
#pragma HLS INTERFACE s_axilite port=return

	int i,flag;
	//总位宽32位，整数9位的有符号定点数--2^9=512(-256<ap_fixed<255)-->360(-180<theta<180)
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> current_x = x;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> temp_x,di;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> current_theta = 0;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> current_y = y;
	ap_fixed<W, I, AP_RND, AP_WRAP, 1> factor = 1.0;

	//x位于负半轴时，先将x取反，后面再根据y的正负与180°相加减-用flag标志出
	if(current_x < 0){
		current_x=-current_x;
		if(current_y<0){
			flag=1;
		}else{
			flag=2;
		}
	}else{
		flag = 0;
	}

	for(i=0;i<NO_ITER;i++){
		//根据y的正负，对【坐标系/对向量】进行顺/逆时针旋转
		di = (current_y < 0) ? 1 : -1;
#pragma HLS LOOP_TRIPCOUNT min=1 max=16
#pragma HLS pipeline
		temp_x = current_x;
		current_x = current_x - di*current_y*factor;
		current_y = current_y + di*temp_x*factor;
		current_theta = current_theta - di*cordic_phase[i];
		//右移一位，进行下一个角度的旋转
		factor = factor >> 1;

	}
	//得到的旋转结果需要除以k
	*r = (float)current_x*0.607252935;
	
	if(flag==1){
		current_theta = -180-current_theta;
	}else if(flag==2){
		current_theta = 180-current_theta;
	}else{
		current_theta = current_theta;
	}
	//将角度转为弧度制
	*theta = (float)current_theta*0.0174533;
}
