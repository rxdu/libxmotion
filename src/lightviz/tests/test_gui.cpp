#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>

#include <FL/Fl_Shared_Image.H>
#include <FL/Fl_JPEG_Image.H>

int main(int argc, char **argv)
{
    fl_register_images();

    Fl_Window *window = new Fl_Window(340, 180);

    window->begin();

    Fl_Box *box = new Fl_Box(20, 40, 300, 100, "Hello, World!");
    box->box(FL_UP_BOX);
    box->labelfont(FL_BOLD + FL_ITALIC);
    box->labelsize(36);
    box->labeltype(FL_SHADOW_LABEL);

    Fl_Box box2(10, 10, 720 - 20, 486 - 20);         // widget that will contain image
    Fl_JPEG_Image jpg("/home/rdu/Pictures/cat.jpg"); // load jpeg image into ram
    box2.image(jpg);                                 // attach jpg image to box

    window->end();

    window->show(argc, argv);
    return Fl::run();
}