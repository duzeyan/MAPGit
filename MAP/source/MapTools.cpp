#include"MapTools.h"
#include<cassert>


////
/// MAP模块工具类
///




void  MapTools::Node2ButtonNode(MAP_NODE &node,MAP_BUTTON_NOTE &buttonNode){
		buttonNode.idself = node.idself;
		buttonNode.neigh = node.neigh;
		int j;
		for (j = 0; j<buttonNode.neigh; j++)
		{
			buttonNode.NeighNoteID[j] = node.NeighNoteID[j];
			buttonNode.NeighLineID[j] = node.NeighLineID[j];
		}
		buttonNode.HLD = node.HLD;
		buttonNode.HLDkind = node.HLDkind;
		buttonNode.lukou = node.lukou;
		buttonNode.zebra = node.zebra;
		buttonNode.gpsx = node.gpsx;
		buttonNode.gpsy = node.gpsy;
		buttonNode.earthx = node.earthx;
		buttonNode.earthy = node.earthy;
}


void  MapTools::Line2ButtonLine(MAP_ROAD &line,MAP_BUTTON_LINE &buttonLine){
		buttonLine.idself = line.idself;
		buttonLine.idstart = line.idstart;
		buttonLine.idend = line.idend; //ID
		buttonLine.k = line.k;           //直线参数
		buttonLine.b = line.b;
		buttonLine.c = line.c;
		buttonLine.roadkind = line.roadkind; 
		buttonLine.wedth = line.wedth;
		buttonLine.length = line.length;
		buttonLine.maluyazi = line.maluyazi;
		buttonLine.hyazi = line.hyazi;
		buttonLine.hulan = line.hulan;
		buttonLine.hhulan = line.hhulan;
		buttonLine.xingdaoxiannum = line.xingdaoxiannum;
		buttonLine.leftxingdaoxian = line.leftxingdaoxian;
		buttonLine.middlexingdaoxian = line.middlexingdaoxian;
		buttonLine.rightxingdaoxian = line.rightxingdaoxian;
		buttonLine.chedaonum = line.chedaonum;
		buttonLine.leftdaolubianjie = line.leftdaolubianjie;
		buttonLine.rightdaolubianjie = line.rightdaolubianjie;
		buttonLine.idealspeed = line.idealspeed;
}

void MapTools::ms_sleep( unsigned int msecs ){
#ifdef _NJUST_OS_USE_LINUX_
	int rc;
	struct timeval tv;
	tv.tv_sec = msecs / 1000;
	tv.tv_usec = ( msecs % 1000 ) * 1000;
	do
	{
		rc = select( 0, NULL, NULL, NULL, &tv );
	}
	while ( rc == -1 && errno == EINTR );
#endif

#ifdef _NJUST_OS_USE_WINDOWS_
	Sleep(msecs);
#endif
}

bool MapTools::CheckGPS(double lng,double lat,double lastlng,double lastlat){
	if (lat>3 && lat<53&& lng>73 && lng<130)
		if(abs(lastlat-lat)<0.0001&&abs(lastlng-lng)<0.0001) //10m
			return true;
	return false;
}

void MapTools::GPS2Earthy(double x, double y, int &earthx, int &earthy)//纬度，经度，单位是度,单位是厘米
{
	int n, L0;
	double X, N54, W54, t, m, a54, e54, e_54;
	double iptr;
	double t_2 = 0, t_4 = 0, yita_2 = 0, yita_4 = 0;
	double lp = 0, lp_2 = 0;
	double SinL, CosL, CosL_2, SinL_2;
	double SinG, CosG;
	double daa, df, db2p, dl2p;
    //double	dahm;
	double deltabo, deltalo;
	double w84, n84, m84, a84, e842, f84, f54, dx, dy, dz;
	double lati, logi;
	double pi = 3.1415926535;

	lati = x;
	logi = y;
	lati = lati*pi / 180;
	logi = logi*pi / 180;
	SinL = sin(lati);
	CosL = cos(lati);
	SinG = sin(logi);
	CosG = cos(logi);
	CosL_2 = CosL * CosL;
	SinL_2 = SinL * SinL;
	a84 = 6378137.0;
	e842 = 0.00669437999014132;
	f84 = 1.0 / 298.257223563;
	a54 = 6378245.0;
	f54 = 1.0 / 298.3;
	dx = -16.0;
	dy = 147.0;
	dz = 77.0;
	w84 = sqrt(1 - e842*SinL_2);
	n84 = a84 / w84;
	m84 = a84*(1 - e842) / (w84*w84*w84);
	daa = a54 - a84;
	df = f54 - f84;
	db2p = (-dx*SinL*CosG - dy*SinL*SinG + dz*CosL + (a84*df + f84*daa)*sin(2 * lati)) / (m84*sin(1 / 3600.0*pi / 180));
	dl2p = (-dx*SinG + dy*CosG) / (n84*CosL*sin(1 / 3600.0*pi / 180));
	//dahm = dx*CosL*CosG + dy*CosL*SinG + dz*SinL + (a84*df + f84*daa)*SinL_2 - daa;
	deltabo = (db2p / 3600.0)*pi / 180.0;
	deltalo = (dl2p / 3600.0)*pi / 180.0;
	logi = logi + deltalo;
	lati = lati + deltabo;
	SinL = sin(lati);
	CosL = cos(lati);
	CosL_2 = CosL * CosL;
	SinL_2 = SinL * SinL;
	a54 = 6378245.0;
	e54 = 0.0066934274898192;
	W54 = sqrt(1.0 - e54*SinL_2);
	N54 = a54 / W54;
	e_54 = 0.0067385254147;
	logi = logi * 180 / pi;
	modf(logi / 6.0, &iptr);
	n = (int)iptr + 1;
	L0 = n * 6 - 3;
	lp = (logi - L0)*pi / 180;
	lp_2 = lp*lp;
	m = CosL_2*lp_2;
	yita_2 = e_54*CosL_2;
	yita_4 = yita_2 * yita_2;
	t = tan(lati);
	t_2 = t*t;
	t_4 = t_2*t_2;
	double gps_xx, gps_yy;
	X = 111134.8611*lati * 180 / pi
		- SinL*CosL*(32005.7799 + 133.9238*SinL_2 + 0.6973*SinL_2*SinL_2 + 0.0039*SinL_2*SinL_2*SinL_2);
	gps_yy = X + N54*t*m*(0.5 + 1.0 / 24.0*(5.0 - t_2 + 9.0*yita_2 + 4.0*yita_4)*m
		+ 1.0 / 720.0*(61.0 - 58.0*t_2 + t_4)*m*m);
	gps_xx = N54*CosL*lp*(1.0 + 1.0 / 6.0*(1 - t_2 + yita_2)*m
		+ 1.0 / 120.0*(5.0 - 18.0*t_2 + t_4 + 14.0*yita_2 - 58.0*yita_2*t_2)*m*m);
	gps_xx = gps_xx + 1000000 * n + 500000;
	gps_xx *= 100;
	gps_yy *= 100;
	earthx = (int)gps_xx;
	earthy = (int)gps_yy;
	return;
}

double MapTools::GetDistanceByGPS(double lng1,double lat1,double lng2,double lat2){
	int earthxcur, earthycur;
	int earthxtemp, earthytemp;
	MapTools::GPS2Earthy(lat1 , lng1 , earthxcur, earthycur);
	MapTools::GPS2Earthy(lat2, lng2, earthxtemp, earthytemp);
	double x = abs(earthxcur - earthxtemp) / 100;
	double y = abs(earthycur - earthytemp) / 100;
	double distance = sqrt(x*x + y*y);
	return distance;
}

int MapTools::GetNodeIndexByID(const vector<MAP_BUTTON_NOTE> &nodes,int id){
	for(size_t i=0;i<nodes.size();i++){
		if(nodes[i].idself==id)
			return i;
	}
	return -1;
}

MAP_BUTTON_NOTE MapTools::GetNodeByID(const vector<MAP_BUTTON_NOTE> &nodes,int id){
	int index=MapTools::GetNodeIndexByID(nodes,id);
	assert(index!=-1);
	return nodes[index];
}

MAP_BUTTON_LINE  MapTools::GetLineByID(const vector<MAP_BUTTON_LINE> &lines,int id){
	int index=MapTools::GetLineIndexByID(lines,id);
	assert(index!=-1);
	return lines[index];
}


int  MapTools::GetLineIndexByID(const vector<MAP_BUTTON_LINE> &lines,int id){
	for(size_t i=0;i<lines.size();i++){
		if(lines[i].idself==id)
			return i;
	}
	return -1;
}

void MapTools::GetObsByDistance(double lng,
								double lat,
								double distanceM,
								const vector<NJUST_MAP_OBSTACLE> &obs,
								vector<NJUST_MAP_OBSTACLE> &outObs){
	double dis=0.0;
	//Step.1 遍历找到所有障碍物中符合条件的
	for(size_t i=0;i<obs.size();i++){
		dis=MapTools::GetDistanceByGPS(lng,lat,
									obs[i].ObstacleCenterGPS.longtitude,
									obs[i].ObstacleCenterGPS.latitude);
		if(dis<distanceM){
			outObs.push_back(obs[i]);
		}
	}
}



void MapTools::StructTransformLine(MAP_BUTTON_LINE* line, NJUST_MAP_INFO_ROAD **road){
	(*road)->roadNum = line->idself - START_LINE_ID + 1;
	switch (line->roadkind)
	{
	case 0:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_NONE;
		break;
	case 1:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_ASPHALT;
		break;
	case 2:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_CEMENT;
		break;
	case 3:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_DIRT;
		break;
	case 4:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_COBBLED;
		break;
	case 5:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_HIGHWAY;
		break;
	case 6:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_BRIDGE;
		break;
	case 7:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_TUNNEL;
		break;
	case 8:
		(*road)->roadType = NJUST_MAP_ROAD_TYPE_CULVERT;
		break;
	}
	switch (line->leftxingdaoxian)
	{
	case 0:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_NONE;
		break;
	case 1:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITEDOTTEDLINE;
		break;
	case 2:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITESOLIDLINE;
		break;
	case 3:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWDOTTEDLINE;
		break;
	case 4:
		(*road)->leftLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWSOLIDLINE;
		break;

	}
	switch (line->middlexingdaoxian)
	{
	case 0:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_NONE;
		break;
	case 1:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITEDOTTEDLINE;
		break;
	case 2:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITESOLIDLINE;
		break;
	case 3:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWDOTTEDLINE;
		break;
	case 4:
		(*road)->centerLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWSOLIDLINE;
		break;

	}
	switch (line->rightxingdaoxian)
	{
	case 0:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_NONE;
		break;
	case 1:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITEDOTTEDLINE;
		break;
	case 2:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_WHITESOLIDLINE;
		break;
	case 3:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWDOTTEDLINE;
		break;
	case 4:
		(*road)->rightLaneLineType = NJUST_MAP_LANE_LINE_TYPE_YELLOWSOLIDLINE;
		break;
	}
	switch (line->leftdaolubianjie)
	{
	case 0:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_NONE;
		break;
	case 1:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONVEXBOUNDARY;
		break;
	case 2:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONCAVEBOUNDARY;
		break;
	case 3:
		(*road)->leftRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_DANGEROUSBOUNDARY;
		break;
	}

	switch (line->rightdaolubianjie)
	{
	case 0:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_NONE;
		break;
	case 1:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONVEXBOUNDARY;
		break;
	case 2:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_CONCAVEBOUNDARY;
		break;
	case 3:
		(*road)->rightRoadBoundaryType = NJUST_MAP_ROAD_BOUNDARY_TYPE_DANGEROUSBOUNDARY;
		break;
	}
	(*road)->nLaneNum = line->chedaonum; //编号
	(*road)->roadWidthCm = int(line->wedth); //TODO 无效
	(*road)->curbWidthCm = int(line->hyazi); //TODO 马路牙子高度
	(*road)->idealSpeedKMH = line->idealspeed;
	(*road)->structure=NJUST_MAP_STRUCTURED_ROAD; //默认结构化道路
	(*road)->GPSDataFrom=NJUST_MAP_GPS_FROM_CAR; //GPS来源
	


	(*road)->nSize = sizeof(NJUST_MAP_INFO_ROAD);
	(*road)->checksum = 0;
}

int MapTools::NJUST_MAP_Encode_IP_Data(const void* pUnknow, int date, char globle[])
{
		char *pdata = (char *)pUnknow;
		globle[0]='#';
		switch (date)
		{
		case 0://road信息
		{
				   globle[1] = '0';
				   memcpy(&globle[2], pdata, sizeof(NJUST_MAP_INFO_ROAD));
				   break;
		}
		case 1://node信息
		{
				   globle[1] = '1';
				   memcpy(&globle[2], pdata, sizeof(NJUST_MAP_INFO_NODE));
				   break;
		}
		case 2://方向导引
		{
				   globle[1] = '2';
				   memcpy(&globle[2], pdata, sizeof(NJUST_MAP_INFO_DIRECTION));
				   break;
		}
		}
		return 0;
}

void MapTools::StructTransformNote(MAP_BUTTON_NOTE* note, NJUST_MAP_INFO_NODE **node)
{
	(*node)->nodeNum = note->idself - START_NODE_ID + 1;
	(*node)->xCM = note->earthx;
	(*node)->yCM = note->earthy;
	switch (note->lukou)
	{
	case 0:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_NONE;
		break;
	case 1:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_CROSSROADS;
		break;
	case 2:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_TNODE;
		break;
	case 3:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_YNODE;
		break;
	case 4:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_RIGHTANGLENODE;
		break;
	case 5:
		(*node)->nodeType = NJUST_MAP_NODE_TYPE_STRAIGHTLINENODE;
		break;
	}
	switch (note->HLD)
	{
	case 0:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_NONE;
		break;
	case 1:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_RIGHTFRONT;
		break;
	case 2:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_LEFTRONT;
		break;
	case 3:
		(*node)->trafficLightsPosition = NJUST_MAP_TRAFFIC_LIGHTS_POSITION_FRONT;
		break;
	}
	switch (note->HLDkind)
	{
	case 0:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_NONE;
		break;
	case 1:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTSTRAIGHTRIGHT;
		break;
	case 2:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTSTRAIGHT;
		break;
	case 3:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_RIGHTSTRAIGHT;
		break;
	case 4:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_LEFTRIGHT;
		break;
	case 5:
		(*node)->trafficLightsType = NJUST_MAP_TRAFFIC_LIGHTS_TYPE_STRAIGHT;
		break;
	}
	
	(*node)->zebraCrossing = note->zebra;
	(*node)->zebraCrossing = note->zebra;
	(*node)->GPSProprity=NJUST_MAP_GPS_FROM_CAR; //GPS来源
	(*node)->nSize = sizeof(NJUST_MAP_INFO_NODE);
	(*node)->checksum = 0;
}

double  MapTools::GetRotateAngle(double x1, double y1, double x2, double y2)//以度为单位
{
		const double epsilon = 1.0e-6;
		const double nyPI = acos(-1.0);
		double dist, dot, degree, angle;
		// normalize
		dist = sqrt(x1 * x1 + y1 * y1);
		x1 /= dist;
		y1 /= dist;
		dist = sqrt(x2 * x2 + y2 * y2);
		x2 /= dist;
		y2 /= dist;
		// dot product
		dot = x1 * x2 + y1 * y2;
		if (fabs(dot - 1.0) <= epsilon)
			angle = 0.0;
		else if (fabs(dot + 1.0) <= epsilon)
			angle = nyPI;
		else {
			double cross;
			angle = acos(dot);	
			cross = x1 * y2 - x2 * y1;
			if (cross < 0) {
				angle = 2 * nyPI - angle;
			}
		}
		degree = angle *  180.0 / nyPI;
		return degree;
}

int MapTools::ID2Code(int id){
	assert(id>=START_NODE_ID);
	if(id<START_LINE_ID){ //是节点ID
		return id-START_NODE_ID+1;
	}
	return id-START_LINE_ID+1;
}

int MapTools::Code2ID(int code,int type){
	assert(code>=0);
	switch (type)
	{
	case 1:
		return code-1+START_NODE_ID;
	case 2:
		return code-1+START_LINE_ID;
	default:
		break;
	}
	return -1;
}
