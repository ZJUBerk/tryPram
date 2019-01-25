xml = [[
<ui closeable="true" on-close="closeEventHandler" resizable="true">
	<label text="Car control pan" wordwrap="true" />
	<group>
		<label text="Simulation time:" id = "999"  wordwrap="false" />
		
	</group>
	<group>
		<label text="Right wheel speed:0" id = "1000" wordwrap="true" />
		<label text="Left wheel speed:0" id = "1001" wordwrap="true" />
		<stretch />
	</group>
	
</ui>
]]


function Right_speed_set(ui,id,newVal)
	simUI.setLabelText(ui,1000,'Right wheel speed:'..newVal)
    Right_vel = newVal
end
function Left_speed_set(ui,id,newVal)
	simUI.setLabelText(ui,1001,'Left wheel speed:'..newVal)
    Left_vel = newVal
end

function closeEventHandler(h)
    sim.addStatusbarMessage('Window '..h..' is closing...')
    simUI.hide(h)
end

function sum_table(a,b,max_bound)
    c={}
	for i in ipairs(a) do
		c[i] = a[i]+b[i]
		if c[i]>max_bound then
			c[i] = max_bound
		elseif c[i]<-max_bound then
			c[i] = -max_bound
		end
	end
	return c
end

if (sim_call_type==sim.syscb_init) then
	MotorHandle_Left=sim.getObjectHandle('LeftMotor')
	MotorHandle_Right=sim.getObjectHandle('RightMotor')
	tip_handle=sim.getObjectHandle('tip')
    tar_handle=sim.getObjectHandle('tar')
	ui=simUI.create(xml)
	startTime=sim.getSimulationTime()
	
	
	Left_vel = 0
	Right_vel = 0
	Start_flag = true
	curtime = 0
    state =0
	
	Dpos = {0,0,0}
	Dpos_sum = {0,0,0}
	Dangle = {0,0,0}
	Dangle_sum = {0,0,0}
	
end

if (sim_call_type==sim.syscb_actuation) then
	curtime=sim.getSimulationTime()-startTime
	
	Dpos = sim.getObjectPosition(tar_handle,tip_handle)
	
	d= math.sqrt( Dpos[1]^2 + Dpos[2]^2 )
	theta = math.atan(Dpos[2],Dpos[1])
	Dangle = sim.getObjectOrientation(tar_handle,tip_handle)
    Dpos_sum = sum_table(Dpos_sum,Dpos,20) 
	Dangle_sum = sum_table(Dangle_sum,Dangle,5) 
	Vx = Dpos[1]*2 + Dpos_sum[1]*0.01
	Vy = Dpos[2]*10 + Dpos_sum[2]*0.01
	Vtheta = Dangle[3]*5 + Dangle_sum[3]*0.0
    --Vtheta = 0
	if state == 0 then
		Left_vel = d*5 - theta * 5
		Right_vel = d*5 + theta * 5
		if math.abs(Dpos[1])<0.03 and math.abs(Dpos[2])<0.03 then
			state = 1
			Dpos_sum = {0,0,0}
		end
	elseif state ==1 then
		Left_vel = -Vtheta * 0.25
		Right_vel =Vtheta * 0.25
		if math.abs(Dangle[3])<1*math.pi/180 then
			state = 2
			Dangle_sum = {0,0,0}
		end
	elseif state ==2 then
		Left_vel=0
		Right_vel = 0
		if math.abs(Dpos[1])>0.04 or math.abs(Dpos[2])>0.04 or math.abs(Dangle[3])>2*math.pi/180 then
			state =0
		end
	end
    sim.setJointTargetVelocity(MotorHandle_Left,Left_vel)
    sim.setJointTargetVelocity(MotorHandle_Right,Right_vel)
    simUI.setLabelText(ui,1000,'Right wheel speed:'..Dpos_sum[1])
    simUI.setLabelText(ui,1001,'Left wheel speed:'..Dpos_sum[2])
	
end

if (sim_call_type==sim.syscb_sensing) then
	simUI.setLabelText(ui,999,'Simulation time:'..curtime)
end

if (sim_call_type==sim.syscb_cleanup) then
	simUI.destroy(ui)
end