#include "path.h"
#include "feedback.h"
#include "control.h"

volatile Path path[2] = {{0}, {0}}; //Shadow memory

//Path information
volatile bool next_path_required = false;
volatile s32 next_path_pos = 0;
volatile s32 next_path_max_v = 0;
volatile s32 next_path_acc = 0;
volatile u8 curr_path = 0;
volatile u8 pend_path = 0;
volatile bool path_static = false;

Path* gen_continuous_path(const s32 sn, const s32 v_max, const s32 acc){
	return gen_path(path[curr_path].tar_vel, path[curr_path].tar_pos, 
						path[curr_path].tar_vel_r, path[curr_path].tar_pos_r, 
						sn, v_max, acc);
}

Path* gen_path(s32 v0, s32 s0, s32 vr, s32 sr, s32 sn, s32 v_max, s32 acc){
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
	
	//If u^2 > 2as
	//This part is rather relaxed and doesn't have the subtle adjustments you will see below
	if ( (SIGN(v0) == SIGN(ds)) && (v0_sqr > ((s64)acc_mult_2 * ABS(ds)))){
		//Overshoot is unavoidable T_T
		//try to stop first and ask for help, which means generating another path
		next_path_required = true;
		next_path_pos = sn;
		next_path_max_v = v_max;
		next_path_acc = acc;
		
		const u8 this_path = !pend_path;
		#define path path[this_path]
		
		__disable_irq();
		path.t1 = 0;
		path.t2 = 0;
		path.t3 = ABS(v0) / acc;
		if (ds > 0){
			path.nom_acc = acc;
			path.dir = DIR_POS;
		}else if(ds < 0){
			path.nom_acc = -acc;
			path.dir = DIR_NEG;
		}else{
			path.nom_acc = acc;
			path.dir = DIR_NEU;
		}
		
		path.seg_acc = path.nom_acc;
		path.t1_pt = path.t2_pt = s0;
		path.end_pt = s0 + (v0 * path.t3 / CONTROL_FREQ / 2);
		path.itr = 0;
		path.vt = v0;
		path.ve = 0;
		
		pend_path = this_path;
		__enable_irq();
		
		#undef path
		//End generation
		return (Path*)&path[pend_path];
	}
	
	/**
	 * A whole lot of subtle vel/acc adjustment adjustment is done here.
	 * Read the wiki for detail.
	 * */
	
	//Predicated vel needed for min. distance traveled in acc. and dec. phases
	const s32 tri_vel = Sqrt((s64)acc_mult_2*ABS(ds) + v0_sqr)/1448; //1448 = sqrt(2)*1024
	if (v_max >= tri_vel){
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
		//Trapezium path, smoothen acceleration and deceleration, NOT velocity
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
		seg_mid = t_mid * vt; //Still scaled by @CONTROL_FREQ

		seg2 = ds*CONTROL_FREQ - seg1 - seg_mid;
		bak_acc = -(vt * vt) / (seg2 / CONTROL_FREQ) / 2;

		t1_pt = s0 + seg1/CONTROL_FREQ;
		t2_pt = t1_pt + seg_mid/CONTROL_FREQ;

		t2 = t1 + t_mid;
		t3 = t2 + (v_max * CONTROL_FREQ + ABS(bak_acc) - 1) / ABS(bak_acc);
	}
	
	//Apply temp variable to actual path
	const u8 this_path = !pend_path;
	
	#define path path[this_path]
	
	__disable_irq();
	//Yeah this is kind of stupid, I know. But it just works.
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.vt = vt;
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	path.bak_acc = bak_acc;
	
	path.t1 = t1;
	path.t2 = t2;
	path.t3 = t3;
	
	path.t1_pt = t1_pt;
	path.t2_pt = t2_pt;
	path.end_pt = sn;
	path.ve = 0;
	path.itr = 0;
	next_path_required = false;
	
	if (ds > 0){
		path.dir = DIR_POS;
	}else if (ds < 0){
		path.dir = DIR_NEG;
	}else{
		path.dir = DIR_NEU;
	}
	
	pend_path = this_path;
	__enable_irq();
	#undef path
	
	return (Path*)&path[pend_path];
}

Path* gen_continuous_const_vel(const s32 vt, const s32 acc){
	return gen_const_vel(path[curr_path].tar_vel, path[curr_path].tar_pos, 
						path[curr_path].tar_vel_r, path[curr_path].tar_pos_r, 
						vt, acc);
}

Path* gen_const_vel(s32 v0, s32 s0, s32 vr, s32 sr, s32 vt, s32 acc){
	const u8 this_path = !pend_path;
	next_path_required = false;
	
	s32 nom_acc;
	if (vt > v0){
		nom_acc =  acc;
	}else{
		nom_acc = -acc;
	}
	
	u32 t1 = (s64)(vt - v0) * CONTROL_FREQ / nom_acc;
	s32 pt = ((s64)vt+v0)*t1/CONTROL_FREQ/2 + s0;
	
	#define path path[this_path]
	
	__disable_irq();
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	
	if (vt > 0){
		path.dir = DIR_POS;
	}else if(vt < 0){
		path.dir = DIR_NEG;
	}else{
		path.dir = DIR_NEU;
	}
	
	path.t1 = t1;
	path.t2 = t1;
	path.t3 = t1;
	path.vt = vt;
	path.ve = vt;
	path.end_pt = path.t1_pt = path.t2_pt = pt;
	path.itr = 0;
	
	pend_path = this_path;
	__enable_irq();
	#undef path
	
	return (Path*)&path[pend_path];
}

Path* path_iterate(){
	//Change to next path using shadow variable
	if (pend_path != curr_path){
		//Rough seas ahead, crew. Strap me to the mizzen when I give the word.
		curr_path = pend_path;
		path_static = false;
	}
	
	#define path path[curr_path]
	
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
		
		path.tar_vel += (path.bak_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.bak_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		
		path.seg_acc = path.bak_acc;
		
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
	#undef path
	
	return (Path*)&path[curr_path];
}

//Reset the path, lock the motor in position
void path_reset(){
	__disable_irq();
	const u8 this_path = !pend_path;
	path[this_path].t1 = path[this_path].t2 = path[this_path].t3 = 0;
	path[this_path].itr = 1;
	path[this_path].tar_pos = get_cnt();
	path[this_path].t1_pt = path[this_path].t2_pt = path[this_path].end_pt = path[this_path].tar_pos;
	path[this_path].tar_vel = path[this_path].tar_vel_r = path[this_path].tar_pos_r = 0;
	path[this_path].vt = path[this_path].ve = 0;
	path[this_path].dir = DIR_NEU;
	path[this_path].nom_acc = path[this_path].seg_acc = MAX_ORIG_ACC;
	path_static = true;
	curr_path = this_path;
	__enable_irq();
}

//To check if the path has finished running or not [That it will not change velocity anymore]
bool is_path_running(){
	return path[curr_path].itr < path[curr_path].t3;
}

u8 get_curr_path(){
	return curr_path;
}

s32 get_path_vel_scaled(){
	return path[curr_path].tar_vel*CONTROL_FREQ + path[curr_path].tar_vel_r;
}

s64 get_path_pos_scaled(){
	return (s64)path[curr_path].tar_pos*(s64)CONTROL_FREQ + path[curr_path].tar_pos_r;
}

s32 get_path_vel(){
	return path[curr_path].tar_vel;
}

s32 get_path_pos(){
	return path[curr_path].tar_pos;
}

u32 get_t1(){
	return path[curr_path].t1;
}

u32 get_t2(){
	return path[curr_path].t2;
}

u32 get_t3(){
	return path[curr_path].t3;
}

s32 get_vt(){
	return path[curr_path].vt;
}

s32 get_ve(){
	return path[curr_path].ve;
}

u32 get_itr(){
	return path[curr_path].itr;
}

u8 get_path_dir(){
	return path[curr_path].dir;
}

bool get_next_required(){
	return next_path_required;
}

bool is_path_static(){
	return path_static;
}

