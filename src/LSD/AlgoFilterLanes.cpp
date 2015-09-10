#include "LaneDetection.h"
#include <fstream>


#define EPS_HERE 1.0e-3
#define INFINI_HERE 1.0e6

ofstream fout_2("pairs.txt");


//(p1-p2)*(p1-p2)
inline double pow_dist_p2p(Point2d p1, Point2d p2)
{
	return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

//(p1-p0)x(p2-p0)
inline double cross(Point2d p0, Point2d p1, Point2d p2)
{
	return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

Rect2d getBoundingBox_d(Point2d p0, Point2d p1, Point2d p2, Point2d p3)
{
	double tf_x = p0.x, tf_y = p0.y;
	double rb_x = p0.x, rb_y = p0.y;
	if (p1.x < tf_x) tf_x = p1.x;
	if (p1.x > rb_x) rb_x = p1.x;
	if (p1.y < tf_y) tf_y = p1.y;
	if (p1.y > rb_y) rb_y = p1.y;

	if (p2.x < tf_x) tf_x = p2.x;
	if (p2.x > rb_x) rb_x = p2.x;
	if (p2.y < tf_y) tf_y = p2.y;
	if (p2.y > rb_y) rb_y = p2.y;

	if (p3.x < tf_x) tf_x = p3.x;
	if (p3.x > rb_x) rb_x = p3.x;
	if (p3.y < tf_y) tf_y = p3.y;
	if (p3.y > rb_y) rb_y = p3.y;

	return Rect2d(tf_x, tf_y, rb_x - tf_x, rb_y - tf_y);
}

Rect getBoundingBox(Point2d p0, Point2d p1, Point2d p2, Point2d p3)
{
	double tf_x = p0.x, tf_y = p0.y;
	double rb_x = p0.x, rb_y = p0.y;
	if (p1.x < tf_x) tf_x = p1.x;
	if (p1.x > rb_x) rb_x = p1.x;
	if (p1.y < tf_y) tf_y = p1.y;
	if (p1.y > rb_y) rb_y = p1.y;

	if (p2.x < tf_x) tf_x = p2.x;
	if (p2.x > rb_x) rb_x = p2.x;
	if (p2.y < tf_y) tf_y = p2.y;
	if (p2.y > rb_y) rb_y = p2.y;

	if (p3.x < tf_x) tf_x = p3.x;
	if (p3.x > rb_x) rb_x = p3.x;
	if (p3.y < tf_y) tf_y = p3.y;
	if (p3.y > rb_y) rb_y = p3.y;

	return Rect(tf_x, tf_y, rb_x - tf_x + 0.5, rb_y - tf_y + 0.5);
}

//distance between two points
inline double dist_p2p(Point2d p1, Point2d p2)
{
	return sqrt(pow_dist_p2p(p1, p2));
}

inline bool adjacent_p2p(Point2d p1, Point2d p2){
	return ((abs(p1.x - p2.x) < 2) && (abs(p1.y - p2.y) < 2));
}

inline double slope_seg(Point2d p1, Point2d p2)
{
	if (abs(p2.x - p1.x) < EPS_HERE) return INFINI_HERE;
	return (p2.y - p1.y) / (p2.x - p1.x);
}

//compute the distance between a point p and a segment (seg1, seg2).
//If the foot is in this segment, all will be ok.
//If not, this fonction will return the smaller one between p, seg1 and p, seg2.
double dist_p2segment(Point2d p, Point2d seg1, Point2d seg2){
	if (abs(seg1.x - seg2.x) < EPS_HERE)
	{
		int val = ((p.y - seg1.y) > 0 ? 1 : -1) * ((p.y - seg2.y) > 0 ? 1 : -1);
		if (val < 0)
			return abs(p.x - seg1.x);
		else
			return sqrt(min(pow_dist_p2p(p, seg1), pow_dist_p2p(p, seg2)));
	}
	if (abs(seg1.y - seg2.y) < EPS_HERE)
	{
		int val = ((p.x - seg1.x) > 0 ? 1 : -1) * ((p.x - seg2.x) > 0 ? 1 : -1);
		if (val < 0)
			return abs(p.y - seg1.y);
		else
			return sqrt(min(pow_dist_p2p(p, seg1), pow_dist_p2p(p, seg2)));
	}

	double k = (seg2.y - seg1.y) / (seg2.x - seg1.x);
	double foot_x = (k * k * seg1.x + k * (p.y - seg1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg1.x) + seg1.y;

	int val = (foot_x - seg1.x > 0 ? 1 : -1) * (foot_x - seg2.x > 0 ? 1 : -1);
	if (val > 0)
	{
		return sqrt(min(pow_dist_p2p(p, seg1), pow_dist_p2p(p, seg2)));
	}
	else
	{
		return dist_p2p(p, Point2d(foot_x, foot_y));
	}
}

double dist_p2line(Point2d p, Segment2d seg)
{
	if (abs(seg.p1.x - seg.p2.x) < EPS_HERE)
	{
		return abs(p.x - seg.p1.x);
	}
	if (seg.p2.y - seg.p1.y < EPS_HERE)
	{
		return abs(p.y - seg.p1.y);
	}

	double k = seg.getSlope();
	double foot_x = (k * k * seg.p1.x + k * (p.y - seg.p1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg.p1.x) + seg.p1.y;

	return dist_p2p(p, Point2d(foot_x, foot_y));
}

bool foot_p2segment(Point2d p, Point2d seg1, Point2d seg2, Point2d &foot)
{
	if (abs(seg1.x - seg2.x) < EPS_HERE)
	{
		int val = ((p.y - seg1.y) > 0 ? 1 : -1) * ((p.y - seg2.y) > 0 ? 1 : -1);
		if (val < 0)
		{
			foot = Point2d(seg1.x, p.y);
			return true;
		}
		else
		{
			return false;
		}
			
	}
	if (abs(seg1.y - seg2.y) < EPS_HERE)
	{
		int val = ((p.x - seg1.x) > 0 ? 1 : -1) * ((p.x - seg2.x) > 0 ? 1 : -1);
		if (val < 0)
		{
			foot = Point2d(p.x, seg1.y);
			return true;
		}
		else
			return false;
	}

	double k = (seg2.y - seg1.y) / (seg2.x - seg1.x);
	double foot_x = (k * k * seg1.x + k * (p.y - seg1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg1.x) + seg1.y;

	int val = (foot_x - seg1.x > 0 ? 1 : -1) * (foot_x - seg2.x > 0 ? 1 : -1);
	if (val < 0)
	{
		foot = Point2d(foot_x, foot_y);
		return true;
	}
	else
	{
		return false;
	}
}

Point2d foot_p2line(Point2d p, Segment2d seg){
	if (abs(seg.p1.x - seg.p2.x) < EPS_HERE)
	{
		return Point2d(seg.p1.x, p.y);
	}
	if (seg.p2.y - seg.p1.y < EPS_HERE)
	{
		return Point2d(p.x, seg.p1.y);
	}

	double k = seg.getSlope();
	double foot_x = (k * k * seg.p1.x + k * (p.y - seg.p1.y) + p.x) / (k * k + 1);
	double foot_y = k * (foot_x - seg.p1.x) + seg.p1.y;
	return Point2d(foot_x, foot_y);
}

Segment2d::Segment2d(Point2d _p1, Point2d _p2)
{
	if (_p1.y < _p2.y)
	{
		p1 = _p1;
		p2 = _p2;
	}
	else
	{
		p2 = _p1;
		p1 = _p2;
	}
	indexZone = (_p1.x + _p2.x) / 2 / 100;
	_ini_slope = false;
	_ini_length = false;
}

void Segment2d::computeSlope(){
	slope = slope_seg(p1, p2);
	_ini_slope = true;
}

void Segment2d::computeLength(){
	length = dist_p2p(p1, p2);
	_ini_length = true;
}

bool Segment2d::isNeighbor(Segment2d s){
	if (abs(indexZone - s.indexZone) > 2)
		return false;
	return true;
}

vector<Segment2d> Segment2d::getValidPoly(Segment2d s, int rows)
{
	vector<Segment2d> vec;

	Point2d foot1 = foot_p2line(p1, s);

	//distance compare
	double thresh = 50;
	if (dist_p2p(foot1, p1) > thresh)
	{
		//fout_2 << foot1 << p1 << thresh << endl;
		//fout_2 << "continue in dist_p2p(foot2, p2) > thresh" << endl;
		return vec;
	}
	Point2d foot2 = foot_p2line(p2, s);
	if (dist_p2p(foot2, p2) > thresh)
	{
		//fout_2 << foot2 << p2 << thresh << endl;
		//fout_2 << "continue in dist_p2p(foot2, p2) > thresh" << endl;
		return vec;
	}
	Segment2d foot_12(foot1, foot2);

	//possible to improve
	if (s.p1.y > foot_12.p2.y || s.p2.y < foot_12.p1.y)
	{
		//fout_2 << p1 << p2 << s.p1 << s.p2 << endl;
		//fout_2 << "continue in s.p1.y > foot_12.p2.y || s.p2.y < foot_12.p1.y" << endl;
		return vec;
	}
		

	//possible to improve
	Point2d arry[4] = { s.p1, foot1, foot2, s.p2 };
	for (int i = 1, j = 0; i < 4; i++)
	{
		Point2d temp = arry[i];
		for (j = i - 1; j >= 0 && arry[j].y >= temp.y; j--)
		{
			arry[j + 1] = arry[j];
		}
		arry[j + 1] = temp;
	}

	//length compare
	Segment2d valid_foot_12(arry[1], arry[2]);
	//if (valid_foot_12.getLength() < s.getLength() * 0.2)
	//{
	//	fout_2 << p1 << p2 << s.p1 << s.p2 << endl;
	//	fout_2 << "continue in valid_foot_12.getLength() < s.getLength() * 0.2" << endl;
	//	return vec;//empty
	//}
		
	
	Point2d new_p1, new_p2;
	
	new_p1 = foot_p2line(valid_foot_12.p1, *this);
	new_p2 = foot_p2line(valid_foot_12.p2, *this);

	//distance compare
	thresh = 40;
	if (new_p1.y + valid_foot_12.p1.y < 8 * rows / 5)
		thresh = 30;
	if (new_p1.y + valid_foot_12.p1.y < 6 * rows / 5)
		thresh = 15;
	if (dist_p2p(new_p1, valid_foot_12.p1) > thresh)
	{
		//fout_2 << new_p1 << valid_foot_12.p1 << thresh << endl;
		//fout_2 << "dist_p2p(new_p1, valid_foot_12.p1) > thresh" << endl;
		return vec;
	}

	thresh = 40;
	if (new_p2.y + valid_foot_12.p2.y < 8 * rows / 5)
		thresh = 30;
	if (new_p2.y + valid_foot_12.p2.y < 6 * rows / 5)
		thresh = 15;
	if (dist_p2p(new_p2, valid_foot_12.p2) > thresh)
	{
		//fout_2 << new_p2 << valid_foot_12.p2 << thresh << endl;
		//fout_2 << "dist_p2p(new_p2, valid_foot_12.p2) > thresh" << endl;
		return vec;
	}

	
	Segment2d new_p12(new_p1, new_p2);
	vec.push_back(new_p12);
	vec.push_back(valid_foot_12);

	return vec;
}

void LaneDetection::AlgoFilterLanes_back(ntuple_list line_out){
	unsigned int dim = line_out->dim;
	int n_size = line_out->size;
	vector< vector<int> > pairs;//each element is a vector of index of segment(s)
	pairs.resize(n_size);

	for (int i = 0; i < n_size; i++)
	{
		const Point2d p1(line_out->values[i * dim + 0], line_out->values[i * dim + 1]);
		const Point2d p2(line_out->values[i * dim + 2], line_out->values[i * dim + 3]);
		double longeur_p12 = dist_p2p(p1, p2);
		Segment2d s(p1, p2);

		//if (line_out->values[i * dim + 0] > 662 && line_out->values[i * dim + 0] < 664)
		//{
		//	int c = 0;
		//}
		//else
		//	continue;

		Point2d milieu_p12 = (p1 + p2) / 2;
		for (int j = 0; j < n_size; j++)
		{
			if (j == i) continue;
			Point2d seg1(line_out->values[j * dim + 0], line_out->values[j * dim + 1]);
			Point2d seg2(line_out->values[j * dim + 2], line_out->values[j * dim + 3]);
			Segment2d seg(seg1, seg2);


			//simple check
			if (!s.isNeighbor(seg))
				continue;

			//not same side
			//if ((cross(p1, seg1, seg2) > -0.0 ? 1 : -1) * (cross(p2, seg1, seg2) > -0.0 ? 1 : -1) < 0)
			//{
			//	continue;
			//}
			
			//slope difference
			double slope_dif = abs(atan(slope_seg(p1, p2)) - atan(slope_seg(seg1, seg2)));
			if (slope_dif > 10 * CV_PI / 180)
			{
				continue;
			}

			double longeur_seg = dist_p2p(seg1, seg2);
			Point2d t_p1, t_p2, t_seg1, t_seg2;
			if (longeur_p12 > longeur_seg * 3)
			{
				if (foot_p2segment(seg1, p1, p2, t_p1) && foot_p2segment(seg2, p1, p2, t_p2))
				{
					t_seg1 = seg1;
					t_seg2 = seg2;
				}
				else
				{
					t_p1 = p1;
					t_p2 = p2;
					t_seg1 = seg1;
					t_seg2 = seg2;
				}
			}
			else if (longeur_seg > longeur_p12 * 3)
			{
				if (foot_p2segment(p1, seg1, seg2, t_seg1) && foot_p2segment(p2, seg1, seg2, t_seg2))
				{
					t_p1 = p1;
					t_p2 = p2;
				}
				else
				{
					t_p1 = p1;
					t_p2 = p2;
					t_seg1 = seg1;
					t_seg2 = seg2;
				}
			}
			else
			{
				t_p1 = p1;
				t_p2 = p2;
				t_seg1 = seg1;
				t_seg2 = seg2;
			}

			Point2d p_start = (t_p1 + t_p2) / 2;
			Point2d p_end = (t_seg1 + t_seg2) / 2;

			//distance
			double thresh = 20;
			if (p_start.y < 2 * processImage.rows / 3 && p_end.y < 2 * processImage.rows / 3)
				thresh = 10;
			if (dist_p2segment(t_p1, t_seg1, t_seg2) > thresh && dist_p2segment(t_p2, t_seg1, t_seg2) > thresh)
			{
				continue;
			}

			//color condition
			int mean_color[3] = { 0, 0, 0 };
			int num[3] = { 0, 0, 0 };
			
			Point2d translation[3];
			translation[0] = Point2d(0, 0);
			translation[1] = p_start - p_end;
			translation[2] = -translation[1];

			for (int _trans = 0; _trans < 3; _trans++)
			{
				Rect box = getBoundingBox(t_p1 + translation[_trans], t_p2 + translation[_trans],
					t_seg1 + translation[_trans], t_seg2 + translation[_trans]);

				Point2d milieu = (p_start + p_end) / 2 + translation[_trans];

				//check direction of cross.
				int direc = (cross(milieu, t_seg1 + translation[_trans], t_seg2 + translation[_trans]) > -EPS_HERE ? 1 : -1) *
					(cross(milieu, t_p1 + translation[_trans], t_p2 + translation[_trans]) > -EPS_HERE ? 1 : -1);
					

				for (int _y = box.y; _y < box.y + box.height; _y++)
				{
					if (_y >= processImage.rows || _y < 0) continue;
					uchar* ptr_row_processImage = ipmImage.ptr<uchar>(_y);
					for (int _x = box.x; _x < box.x + box.width; _x++)
					{
						if (_x >= processImage.cols || _x < 0) continue;
						Point2d p(_x, _y);
						if (direc != (cross(p, t_seg1 + translation[_trans], t_seg2 + translation[_trans]) > -EPS_HERE ? 1 : -1) *
							(cross(p, t_p1 + translation[_trans], t_p2 + translation[_trans]) > -EPS_HERE ? 1 : -1))
						{
							continue;
						}
						mean_color[_trans] += ptr_row_processImage[_x];
						num[_trans]++;
					}
				}
				mean_color[_trans] = (double)mean_color[_trans] / num[_trans];
			}

			bool color_matched = (mean_color[0] > mean_color[1] + 30) && (mean_color[0] > mean_color[2] + 30);

			if (!color_matched)
			{
				//line(processImage, p1, p2, Scalar(255, 0, 0));
				continue;
			}


			Rect box = getBoundingBox(t_p1, t_p2,
				t_seg1, t_seg2);

			Point2d milieu = (p_start + p_end) / 2;
			
			int _trans = 0;
			//check direction of cross.
			int direc = (cross(milieu, t_seg1 + translation[_trans], t_seg2 + translation[_trans]) > -EPS_HERE ? 1 : -1) *
				(cross(milieu, t_p1 + translation[_trans], t_p2 + translation[_trans]) > -EPS_HERE ? 1 : -1);


			for (int _y = box.y; _y < box.y + box.height; _y++)
			{
				if (_y >= processImage.rows || _y < 0) continue;
				Vec3b* ptr_row_processImage = processImage.ptr<Vec3b>(_y);
				for (int _x = box.x; _x < box.x + box.width; _x++)
				{
					if (_x >= processImage.cols || _x < 0) continue;
					Point2d p(_x, _y);
					if (direc != (cross(p, t_seg1 + translation[_trans], t_seg2 + translation[_trans]) > -EPS_HERE ? 1 : -1) *
						(cross(p, t_p1 + translation[_trans], t_p2 + translation[_trans]) > -EPS_HERE ? 1 : -1))
					{
						continue;
					}
					ptr_row_processImage[_x] = Vec3b(255, 0, 255);
				}
			}
			
			//TODO: ADD MORE CONDITIONS to check if they are a pair.
			//line(processImage, p1, p2, Scalar(0, 255, 0));
			//break;
			//add candidate pair to vector.
		}
	}

	ofstream fout("pairs.txt");
	for (int i = 0; i < n_size; i++)
	{
		const Point2d p1(line_out->values[i * dim + 0], line_out->values[i * dim + 1]);
		const Point2d p2(line_out->values[i * dim + 2], line_out->values[i * dim + 3]);

		fout << i << "{" << p1 << "; " << p2 << "}" << " pairs: ";
		int b = (unsigned)theRNG() & 255;
		int g = (unsigned)theRNG() & 255;
		int r = (unsigned)theRNG() & 255;

		if (pairs[i].size() <= 1)
			continue;

		for (int j = 0; j < pairs[i].size(); j++)
		{
			Point2d seg1(line_out->values[pairs[i][j] * dim + 0], line_out->values[pairs[i][j] * dim + 1]);
			Point2d seg2(line_out->values[pairs[i][j] * dim + 2], line_out->values[pairs[i][j] * dim + 3]);



			fout << "[" << seg1 << "; " << seg2 << " ],";
		}




		//line(processImage, p1, p2, Scalar(b, g, r));
		//line(processImage, seg1, seg2, Scalar(b, g, r));

		fout << "--------" << endl;
	}

}

void LaneDetection::AlgoFilterLanes(ntuple_list line_out)
{
	//fout_2.open("pairs.txt");
	unsigned int dim = line_out->dim;
	int n_size = line_out->size;
	
	Mat lines_candidats = Mat::zeros(processGrayImage.size(), CV_32SC1);

	for (int i = 0; i < n_size; i++)
	{
		const Point2d p1(line_out->values[i * dim + 0], line_out->values[i * dim + 1]);
		const Point2d p2(line_out->values[i * dim + 2], line_out->values[i * dim + 3]);
		Segment2d p12 = Segment2d(p1, p2);//p1.y < p2.y

		for (int j = 0; j < n_size; j++)
		{
			if (j == i) continue;
			Point2d seg1(line_out->values[j * dim + 0], line_out->values[j * dim + 1]);
			Point2d seg2(line_out->values[j * dim + 2], line_out->values[j * dim + 3]);
			Segment2d seg(seg1, seg2);
			
			if (seg.getLength() > p12.getLength())
			{
				continue;
			}

			//simple check
			if (!p12.isNeighbor(seg))
			{
				continue;
			}

			//slope difference?
			if (abs(atan(p12.getSlope()) - atan(seg.getSlope())) > 15 * CV_PI / 180)
			{
				//fout_2 << p1 << p2 << seg1 << seg2 << p12.getSlope() << "," << seg.getSlope() << endl;
				//fout_2 << "continue in abs(atan(p12.getSlope()) - atan(seg.getSlope())) > 10 * CV_PI / 180" << endl;
				continue;
			}

			vector<Segment2d> v = p12.getValidPoly(seg, processGrayImage.rows);
			//valid?
			if (v.empty())
			{
				//fout_2 << p1 << p2 << seg1 << seg2 << endl;
				//fout_2 << "continue in v.empty()" << endl;
				continue;
			}

			//color compare
			Point2d translation[3];
			translation[0] = Point2d(0, 0);
			translation[1] = v[0].p1 - v[1].p1;
			translation[2] = -translation[1];

			vector<uchar> vec_color[3];
			double mean_color[3];
			double stdDev_color[3];
			

			for (int _trans = 0; _trans < 3; _trans++)
			{
				Rect box = getBoundingBox(v[0].p1 + translation[_trans], v[0].p2 + translation[_trans],
					v[1].p1 + translation[_trans], v[1].p2 + translation[_trans]);
				for (int _y = box.y; _y < box.y + box.height; _y++)
				{
					if (_y >= processImage.rows || _y < 0) continue;
					uchar* ptr_row_processImage = processGrayImage.ptr<uchar>(_y);
					for (int _x = box.x; _x < box.x + box.width; _x++)
					{
						if (_x >= processImage.cols || _x < 0) continue;
						Point2d p(_x, _y);
						int direc = cross(p, v[0].p1 + translation[_trans], v[0].p2 + translation[_trans]) > -0.0 ? 1 : -1;

						if (direc != (cross(p, v[0].p2 + translation[_trans], v[1].p2 + translation[_trans]) > -0.0 ? 1 : -1))
							continue;
						if (direc != (cross(p, v[1].p2 + translation[_trans], v[1].p1 + translation[_trans]) > -0.0 ? 1 : -1))
							continue;
						if (direc != (cross(p, v[1].p1 + translation[_trans], v[0].p1 + translation[_trans]) > -0.0 ? 1 : -1))
							continue;

						vec_color[_trans].push_back(ptr_row_processImage[_x]);
					}
				}
				if (vec_color[_trans].empty())
					;
				else
				{
					Mat mean, stdDev;
					meanStdDev(vec_color[_trans], mean, stdDev);
					mean_color[_trans] = mean.at<double>(0);
					stdDev_color[_trans] = stdDev.at<double>(0);
				}
			}

			bool color_matched = (mean_color[0] > mean_color[1] + 10) && (mean_color[0] > mean_color[2] + 10)
				&& stdDev_color[0] < 50;

			if (!color_matched)
			{
				//fout_2 << p1 << p2 << seg1 << seg2 << mean_color[0] << "," << mean_color[1] << "," << mean_color[2];
				//fout_2 << "," << stdDev_color[0] << endl;
				//fout_2 << "continue in color_matched" << endl;

				continue;
			}

			//fout_2 << p1 << p2 << seg1 << seg2 << endl;
			//fout_2 << "Paired!!!!!!" << endl;

			line(processImage, p1, p2, Scalar(0, 0, 255));
			line(processImage, seg1, seg2, Scalar(0, 255, 0));
			//imshow("step1", processImage);
			//waitKey();
			//break;
		}//end for j

	}


	//fout_2 << "-------------------" << endl;
	//fout_2.close();
}


