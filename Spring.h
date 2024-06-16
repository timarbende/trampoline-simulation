#pragma once
class Spring {
public:
	int p1;
	int p2;
	float initialLength;
	Spring(int point1, int point2, float initialLen) {
		p1 = point1;
		p2 = point2;
		initialLength = initialLen;
	}
};
