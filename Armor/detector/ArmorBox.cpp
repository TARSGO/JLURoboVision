#include "Armor/Armor.h"

/**
 *@brief: calculate the cross point of four points in order bl(below left),tl(top left),tr(top right),br(below right)
 */
const Point2f crossPointof(const Point2f& bl,const Point2f& tl,const Point2f& tr, const Point2f& br){
	float a1 = tr.y - bl.y;
	float b1 = tr.x - bl.x;
	float c1 = bl.x*tr.y - tr.x*bl.y;

	float a2 = br.y - tl.y;
	float b2 = br.x - tl.x;
	float c2 = tl.x*br.y - br.x*tl.y;

	float d = a1 * b2 - a2 * b1;

	if (d == 0.0){
		return Point2f(FLT_MAX, FLT_MAX);
	}
	else{
		return cv::Point2f((b2*c1 - b1 * c2) / d, (c1*a2 - c2 * a1) / d);
	}
}

/**
 *@brief: using the lightRect of two lightBar to construct the armorVertices
 */
void setArmorVertices(const LightBar & l_light, const LightBar & r_light, ArmorBox & armor) {
	//handle two lights
	cv::Size exLSize(int(l_light.width), int(l_light.length));
	cv::Size exRSize(int(r_light.width), int(r_light.length));
	cv::RotatedRect exLLight(l_light.center, exLSize, armor.armorAngle);
	cv::RotatedRect exRLight(r_light.center, exRSize, armor.armorAngle);

	cv::Point2f pts_l[4];
	exLLight.points(pts_l);
	cv::Point2f upper_l = pts_l[2];
	cv::Point2f lower_l = pts_l[3];

	cv::Point2f pts_r[4];
	exRLight.points(pts_r);
	cv::Point2f upper_r = pts_r[1];
	cv::Point2f lower_r = pts_r[0];
	
	armor.armorVertices[0] = upper_l;
	armor.armorVertices[1] = upper_r;
	armor.armorVertices[2] = lower_r;
  armor.armorVertices[3] = lower_l;

}

ArmorBox::ArmorBox() {
	l_index = -1;
	r_index = -1;
	l_light = LightBar();
	r_light = LightBar();
	armorNum = 0;
	armorVertices.resize(4);
	type = ArmorType::SMALL_ARMOR;
	center = Point2f();
	armorImg = Mat();

  distanceDiff = 0.0;
}

ArmorBox::ArmorBox(const LightBar & l_light, const LightBar & r_light) {
	this->l_light = l_light;
	this->r_light = r_light;
	
	armorNum = 0;
	armorAngle = (l_light.angle + r_light.angle) / 2;
	
	//set armorVertices bl->tl->tr->br     左下 左上 右上 右下
	armorVertices.resize(4);
	setArmorVertices(l_light, r_light, *this);    // '*this' means the reference of this ArmorBox

	//set armor center
	center = crossPointof(armorVertices[0], armorVertices[1], armorVertices[2], armorVertices[3]);
}

ArmorBox::~ArmorBox() {}


bool ArmorBox::operator==(ArmorBox& b)
{
	if (this->l_index == b.l_index && this->r_index == b.r_index)
		return 1;
	
	else
		return 0;
}


// dislocation judge X: r-l light center distance ration on the X-axis 灯条位置差距 两灯条中心x方向差距比值
float ArmorBox::getDislocationX() const {
	float meanLen = (l_light.length + r_light.length) / 2;
	float xDiff = abs(l_light.center.x - r_light.center.x); //x distance ration x轴方向上的距离比值（x轴距离与灯条平均值的比）
    float xDiff_ratio = xDiff / meanLen;
	return xDiff_ratio;
}


// judge whether this armor is suitable or not  判断本装甲板是否是合适的装甲板
bool ArmorBox::isSuitableArmor() const
{
    // Ratio of the length of 2 lights (short side / long side)
    double light_length_ratio = l_light.length < r_light.length ? l_light.length / r_light.length
                                                                : r_light.length / l_light.length;
    bool light_ratio_ok = light_length_ratio > 0.7;

    // Distance between the center of 2 lights (unit : light length)
    double avg_light_length = (l_light.length + r_light.length) / 2;
    double center_distance = cv::norm(l_light.center - r_light.center) / avg_light_length;
    bool center_distance_ok = (0.8 < center_distance &&
                               center_distance < 3.2) ||
                              (3.2 < center_distance &&
                               center_distance < 5.0);

    // Angle of light center connection
    cv::Point2f diff = l_light.center - r_light.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < 35.0;
    bool armor_ok = light_ratio_ok & center_distance_ok & angle_ok;
    return armor_ok;
}

                                                                                      