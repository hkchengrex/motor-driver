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
	s32 nom_acc; //Acceleration used in the path, same sign as first segment, cnt/s^-2
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
	
	//Intermediate calculation variables
	const s32 ds = sn - s0; //Delta distance
	const s64 v0_sqr = v0 * v0;
	const s32 acc_mult_2 = 2 * acc;
	
	// //If u^2 > 2as
	// if ( (SIGN(v0) == SIGN(ds)) && (v0_sqr > ((s64)acc_mult_2 * ABS(ds)))){
	// 	//Overshoot is unavoidable T_T
	// 	//try to stop first and ask for help, which means generating another path
	// 	next_path_required = true;
	// 	next_path_pos = sn;
	// 	next_path_max_v = v_max;
	// 	next_path_acc = acc;
		
	// 	const u8 this_path = !pend_path;
	// 	#define path path[this_path]
		
	// 	__disable_irq();
	// 	path.t1 = 0;
	// 	path.t2 = 0;
	// 	path.t3 = ABS(v0) / acc;
	// 	if (ds > 0){
	// 		path.nom_acc = acc;
	// 		path.dir = DIR_POS;
	// 	}else if(ds < 0){
	// 		path.nom_acc = -acc;
	// 		path.dir = DIR_NEG;
	// 	}else{
	// 		path.nom_acc = acc;
	// 		path.dir = DIR_NEU;
	// 	}
		
	// 	path.seg_acc = path.nom_acc;
	// 	path.t1_pt = path.t2_pt = s0;
	// 	path.end_pt = s0 + (v0 * path.t3 / CONTROL_FREQ / 2);
	// 	path.itr = 0;
	// 	path.vt = v0;
	// 	path.ve = 0;
		
	// 	pend_path = this_path;
	// 	__enable_irq();
		
	// 	#undef path
	// 	//End generation
	// 	return (Path*)&path[pend_path];
	// }
	
	//Predicated vel needed for min. distance traveled in acc. and dec. phases
	const s32 tri_vel = Sqrt((s64)acc_mult_2*ABS(ds) + v0_sqr)/1448; //1448 = sqrt(2)*1024

	s32 vt, nom_acc;
	//Determine the shape of the graph
	if (ds > 0) {
		//Forward path
		if (v_max > tri_vel) {
			//Triangle path
			vt = tri_vel;
		}else {
			//Tripezium path
			vt = v_max;
		}
		nom_acc = acc;
		
	}else {
		//Backward path
		if (v_max > tri_vel) {
			//Triangle path
			vt = -tri_vel;
		}
		else {
			//Tripezium path
			vt = -v_max;
		}
		nom_acc = -acc;
	}

	u32 t1, t2, t3;
	//Calculate the critical time instances for the graph
	t1 = (vt - v0) * CONTROL_FREQ / nom_acc;

	//The following are scaled by @CONTROL_FREQ
	s64 seg1 = (s64)(t1) * (vt + v0) / 2;
	s64 seg2 = (s64)(vt) * vt * CONTROL_FREQ / acc_mult_2 * SIGN(vt);
	s64 seg_mid = ((s64)ds*CONTROL_FREQ - seg1 - seg2);

	s32 t1_pt, t2_pt;
	t1_pt = s0 + seg1/CONTROL_FREQ;
	t2_pt = t1_pt + seg_mid/CONTROL_FREQ;
	
	t2 = t1 + seg_mid / (s64)(vt);
	t3 = t2 + ABS(vt) * CONTROL_FREQ / ABS(acc);

	//Yeah this is kind of stupid, I know. But it just works.
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.vt = vt;
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	
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

Path* path_iterate(){
	
	if (path.itr < path.t1){
		//Acceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel  += (path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos  += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		path.seg_acc = path.nom_acc;
		
	}else if(path.itr == path.t1){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t1_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(0);
	}
	
	if(path.itr >= path.t1 && path.itr < path.t2){
		//Constant phase
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		
	}else if(path.itr == path.t2){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t2_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(1);
	}
	
	if(path.itr >= path.t2 && path.itr < path.t3){
		//Deceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel += (-path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (-path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		
		path.seg_acc = -path.nom_acc;
		
	}else if(path.itr == path.t3){
		path.tar_pos = path.end_pt;
		path.tar_pos_r = 0;
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		
		path.seg_acc = 0;
		
		if (path.ve == 0){
			path.dir = DIR_NEU;
		}
		
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
	s32 tar = 4000;
    gen_path(0, 0, 0, 0, tar, 15000, 20000);
	printf("1: %d 2: %d 3: %d\n", path.t1, path.t2, path.t3);

    while(1){
        path_iterate();

        printf("%d %d\n", path.tar_pos, path.tar_vel);

        if (path.tar_pos == tar) break;
    }

    return 0;
}