
#include "Armor/Armor.h"


/**
* @brief: match lights into armors 将识别到的灯条拟合为装甲板
*/
void ArmorDetector::matchArmors(){
	for (int i = 0; i < lights.size() - 1; i++)
	{
		for (int j = i + 1; j < lights.size(); j++) {
            if(containLight(lights[i],lights[j], lights))
            {
                continue;
            }
            ArmorBox armor = ArmorBox(lights[i], lights[j]); //construct an armor using the matchable lights 利用左右灯条构建装甲板
			if (armor.isSuitableArmor()) //when the armor we constructed just now is a suitable one,set extra information of armor 如果是合适的装甲板，则设置其他装甲板信息
            {
                double avg_light_length = (armor.l_light.length + armor.r_light.length) / 2;
                double center_distance = cv::norm(armor.l_light.center - armor.r_light.center) / avg_light_length;
                armor.type = center_distance > 3.2 ? ArmorType::BIG_ARMOR : ArmorType::SMALL_ARMOR;
                armor.l_index = i; //set index of left light 左灯条的下标
                armor.r_index = j; //set index of right light 右灯条的下标
				classifier.getArmorImg(armor);// set armor image 装甲板的二值图
				classifier.setArmorNum(armor);//set armor number 装甲板数字
                if(armor.confidence > 0.7 && armor.armorNum != 'N' && ArmorJudge(armor))
                {
                    armors.emplace_back(armor);
                    classifier.showArmorNum();
                }
			}
		}
        eraseCrossLightsArmor();
        eraseErrorRepeatArmor();//delete the error armor caused by error light 删除游离灯条导致的错误装甲板
	}
	if (armors.empty()/*||armors.size() > 7*/) {
		state = DetectorState::ARMOR_NOT_FOUND;;
        return; //exit function
	} 
	else {
		state = DetectorState::ARMOR_FOUND; //else set state ARMOR_FOUND 如果非空（有装甲板）则设置状态ARMOR_FOUND
		return; //exit function
	}
}

/**
 *@brief: detect and delete error armor which is caused by the single lightBar 针对游离灯条导致的错误装甲板进行检测和删除
 */
void ArmorDetector::eraseErrorRepeatArmor()
{
	int length = armors.size();
	vector<ArmorBox>::iterator it = armors.begin();
    for (size_t i = 0; i < length; i++) {
        for (size_t j = i + 1; j < length; j++)
		{
            if (
                armors[i].l_index == armors[j].l_index ||
                armors[i].r_index == armors[j].r_index ||
                armors[i].l_index == armors[j].r_index ||
                armors[i].r_index == armors[j].l_index
                )
            {
                (armors[i].getDislocationX()>armors[j].getDislocationX())?armors.erase(it + i):armors.erase(it + j);
                length = armors.size();
                it = armors.begin();
            }
        }
	}
}

void ArmorDetector::eraseCrossLightsArmor()
{
	int length = armors.size();
	vector<ArmorBox>::iterator it = armors.begin();
	for (int i = 0; i < length; i++) {
		int l_index = armors[i].l_index;
		int r_index = armors[i].r_index;
		int l_y = lights[l_index].center.y;
		int r_y = lights[r_index].center.y;
		for (int j = l_index + (int)1; j < r_index; j++){
			if (lights[j].center.y < MAX(l_y, r_y) + 10 && lights[j].center.y > MIN(l_y, r_y) - 10) {
				armors.erase(it + i); 
				length = armors.size();
				it = armors.begin();
				break;
			}
		}
	}
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool ArmorDetector::containLight(
        const LightBar & light_1, const LightBar & light_2, const std::vector<LightBar> & lights) {
    auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
    auto bounding_rect = cv::boundingRect(points);

    for (const auto &test_light: lights) {
        if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

        if (
                bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
                bounding_rect.contains(test_light.center)) {
            return true;
        }
    }
    return false;
}

bool ArmorDetector::ArmorJudge(ArmorBox armor)
{
    bool mismatch = false;
    if (armor.type == ArmorType::BIG_ARMOR) {
        mismatch = armor.armorNum == 'O' || armor.armorNum == '2' || armor.armorNum == '3';
    } else if (armor.type == ArmorType::SMALL_ARMOR) {
        mismatch = armor.armorNum == '1' || armor.armorNum == 'B' || armor.armorNum == 'G';
    }
    return !mismatch;
}