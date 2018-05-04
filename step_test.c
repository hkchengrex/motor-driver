#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define s32 int32_t
#define u8 uint8_t
#define u32 uint32_t
#define s64 int64_t

#define CONTROL_FREQ 16

#define DIR_POS 1
#define DIR_NEU 0
#define DIR_NEG 2

#define ABS(x) ((x)<0?(-(x)):(x))
#define SQR(x) ((x)*(x))
#define SIGN(x) ((x)<0?(-1):(1))
#define CAP(a, b, c) ((a)<(b)?(b):((a)>(c)?(c):(a)))

typedef struct{
	s32 nom_acc; // Forward acceleration used in the path, same sign as first segment, cnt/s^-2
	s32 bak_cc; // Backwards acceleration, might be slightly different with @nom_acc due to acceleration smoothing
	s32 seg_acc; //Acceleration in the current segment, correct sign, cnt/s^-2
	s32 vt; //Terminal velocity, correct sign, cnt/s^-1
	s32 ve; //The velocity that it should maintain at the end of the path, cnt/s^-1
	s32 tar_vel; //Target velocity, cnt/s^-1
	s32 tar_pos; //Target position, cnt
	s32 tar_vel_r; //Target velocity remainder, cnt/s^-1
	s32 tar_pos_r; //Target position remainder, cnt
	u32 t1; //End of acceleration phase, in number of iteration
	u32 t2; //End of constant phase, in number of iteration
	u32 t3; //End of path, in number of iteration
	u32 itr; //The current number of iteration
	s32 t1_pt; //Position at t1, cnt, used to re-cali
	s32 t2_pt; //Position at t2, cnt, used to re-cali
	s32 end_pt; //Ending position, cnt, used to re-cali
	u8 dir; //Direction of path, based on the s0, sn and vt
} Path;

Path path;

void pt_arrival_feedback(u8 num){
    //printf("Reached: %d\n", num);
}

s32 Sqrt(s64 num){
    return sqrt((float)num) * 1024;
}

void gen_path(s32 v0, s32 s0, s32 vr, s32 sr, s32 sn, s32 v_max, s32 acc){
	//Safeguard
	v_max = ABS(v_max);
	acc = ABS(acc);
	
	//Intermediate calculation variables
	const s32 ds = sn - s0; //Delta distance
	const s64 v0_sqr = v0 * v0;
	s32 acc_mult_2 = 2 * acc; //Will be smoothed below

	s32 bak_acc = 0;
	s32 t1 = 0, t2 = 0, t3 = 0;
	s32 vt = 0, nom_acc = 0;
	s32 t1_pt = 0, t2_pt = 0;
	
	/**
	 * A whole lot of subtle vel/acc adjustment adjustment is done here.
	 * Read the wiki for detail.
	 * */

	//Predicated vel needed for min. distance traveled in acc. and dec. phases
	s32 tri_vel = Sqrt((s64)acc_mult_2*ABS((s64)ds) + (s64)v0_sqr)/1448; //1448 = sqrt(2)*1024
	if (v_max >= tri_vel){
		printf("Triangle\n");
		vt = tri_vel * SIGN(ds);
		//Need triangle path, smoothen velocity
		//Note that although we can perform the same smoothing operation on tripezium path, we don't
		//so as to provide velocity guarantee which is critical

		//Got optimal tri_vel, now retrace to get time step
		t1 = ((vt - v0)*CONTROL_FREQ) / acc;
		t1 = t2 = ABS(t1);
		//Retrace again for integral velocity
		vt = (t1*acc/CONTROL_FREQ + v0) * SIGN(ds);

		//Smoothen deceleration in similar manner
		s32 accel_ds = ((vt + v0) * t1) / CONTROL_FREQ / 2;
		bak_acc = -(vt * vt) / (ds - accel_ds) / 2;

		//Smooth acceleration enforced by smooth velocity already
		nom_acc = acc * SIGN(ds);
		t3 = t2 + (ABS(vt) * CONTROL_FREQ + ABS(bak_acc) - 1) / ABS(bak_acc);
		t1_pt = t2_pt = accel_ds;
	}else{
		printf("Tripezium\n");
		//Tripezium path, smoothen acceleration and deceleration, NOT velocity
		vt = v_max * SIGN(ds);
		//Smoothen acceleration
		//Min. time steps to accelerate = ceil[max_v*freq/acc]
		//Smoothened accel = max_v * freq / min. step
		s32 abs_dv = ABS(vt - v0);
		nom_acc = SIGN(vt) * abs_dv * CONTROL_FREQ / ((abs_dv * CONTROL_FREQ + acc - 1) / acc);
		acc_mult_2 = 2*ABS(nom_acc);

		t1 = abs_dv * CONTROL_FREQ / acc;
		//The constant velocity phase is the boss. Always respect it.
		//The following are scaled by @CONTROL_FREQ
		s64 seg1 = (s64)(t1) * (vt + v0) / 2;
		s64 seg2 = (s64)(vt) * vt * CONTROL_FREQ / acc / 2 * SIGN(vt); //Use normal acc for now
		//Let's see how far the mid wants to go
		s64 seg_mid = ((s64)ds*CONTROL_FREQ - seg1 - seg2);
		//Trim it to integral distance
		s32 t_mid = seg_mid / vt;
		t_mid = ABS(t_mid);
		printf("tm: %d\n", t_mid);
		seg_mid = t_mid * vt; //Still scaled by @CONTROL_FREQ

		seg2 = ds*CONTROL_FREQ - seg1 - seg_mid;
		bak_acc = -(vt * vt) / (seg2 / CONTROL_FREQ) / 2;

		t1_pt = s0 + seg1/CONTROL_FREQ;
		t2_pt = t1_pt + seg_mid/CONTROL_FREQ;

		t2 = t1 + t_mid;
		t3 = t2 + (v_max * CONTROL_FREQ + ABS(bak_acc) - 1) / ABS(bak_acc);
	}

	printf("%d\n", tri_vel);
	printf("%d\n", nom_acc);

	//Calculate the critical time instances for the graph
	//t1 = ((vt - v0) * CONTROL_FREQ) / nom_acc; 

	printf("%d\n", bak_acc);

	//Yeah this is kind of stupid, I know. But it just works.
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.vt = vt;
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	path.bak_cc = bak_acc;
	
	path.t1 = t1;
	path.t2 = t2;
	path.t3 = t3;
	
	path.t1_pt = t1_pt;
	path.t2_pt = t2_pt;
	path.end_pt = sn;
	path.ve = 0;
	path.itr = 0;
	
	if (ds > 0){
		path.dir = DIR_POS;
	}else if (ds < 0){
		path.dir = DIR_NEG;
	}else{
		path.dir = DIR_NEU;
	}
}

void path_iterate(){
	
	//if (path.itr < (path.t1-1)){
	if (path.itr < (path.t1)){
		//Acceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel  += (path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos  += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		path.seg_acc = path.nom_acc;
		
	// }else if(path.itr == (path.t1-1)){
	// 	//Recali to reduce integration error
	// 	path.tar_vel = path.vt;
	// 	path.tar_vel_r = 0;
	// 	path.tar_pos = path.t1_pt;
	// 	path.tar_pos_r = 0;
	}else if (path.itr == path.t1){
		pt_arrival_feedback(0);
	}
	
	//if(path.itr >= path.t1 && path.itr < (path.t2-1)){
	if(path.itr >= path.t1 && path.itr < (path.t2)){
		//Constant phase
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		
	// }else if(path.itr == (path.t2-1)){
	// 	//Recali to reduce integration error
	// 	path.tar_vel = path.vt;
	// 	path.tar_vel_r = 0;
	// 	path.tar_pos = path.t2_pt;
	// 	path.tar_pos_r = 0;
	}else if (path.itr == path.t2){
		pt_arrival_feedback(1);
	}
	
	if(path.itr >= path.t2 && path.itr < (path.t3-1)){
		//Deceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel += (path.bak_cc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.bak_cc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		
		path.seg_acc = path.bak_cc;
		
	}else if(path.itr == (path.t3-1)){
		path.tar_pos = path.end_pt;
		path.tar_pos_r = 0;
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		
		path.seg_acc = 0;
	}else if (path.itr == path.t3){
		pt_arrival_feedback(2);
	}
	
	if (path.itr >= path.t3){
		//After the end
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		//Lock the iterator
		path.itr = path.t3 + 1;
	}
	path.itr++;
}

int main(){
	s32 tar = 20000;
    gen_path(5000, 10000, 0, 0, tar, 15000, 20000);

	printf("T1: %d 2: %d 3: %d\n", path.t1, path.t2, path.t3);
	printf("S1: %d 2: %d 3: %d\n", path.t1_pt, path.t2_pt, path.end_pt);
    while(1){
		for (int i=0; i<500000; i++){
			i = i;
		}
        path_iterate();

        printf("%d %d\n", path.tar_pos, path.tar_vel);

        if (path.tar_pos == tar) break;
    }

    return 0;
}