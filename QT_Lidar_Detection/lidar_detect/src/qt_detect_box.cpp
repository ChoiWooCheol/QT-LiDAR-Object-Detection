#include "qt_header/qt_detect_node.hpp"
#include "qt_header/qt_detect_params.hpp"

BOX BOX::getUR()
{
    double in_x = this->Xpose + this->size / 4.0;
    double in_y = this->Ypose + this->size / 4.0;
    double in_size = this->size / 2.0;
    BOX box(in_x, in_y, in_size, pixelSize / 2);
    return box;
}

BOX BOX::getUL()
{
    double in_x = this->Xpose - this->size / 4.0;
    double in_y = this->Ypose + this->size / 4.0;
    double in_size = this->size / 2.0;
    BOX box(in_x, in_y, in_size, pixelSize / 2);
    return box;
}

BOX BOX::getDR()
{
    double in_x = this->Xpose + this->size / 4.0;
    double in_y = this->Ypose - this->size / 4.0;
    double in_size = this->size / 2.0;
    BOX box(in_x, in_y, in_size, pixelSize / 2);
    return box;
}

BOX BOX::getDL()
{
    double in_x = this->Xpose - this->size / 4.0;
    double in_y = this->Ypose - this->size / 4.0;
    double in_size = this->size / 2.0;
    BOX box(in_x, in_y, in_size, pixelSize / 2);
    return box;
}