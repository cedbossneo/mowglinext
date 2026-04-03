#include <stdlib.h>
#include "config.h"
#ifdef HAVE_INTTYPES_H
# include <inttypes.h>
#endif
#include <gtk/gtk.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <gio/gunixinputstream.h>

#define NPOINTS 1284

typedef struct oscilloscope* oscilloscope;

struct oscilloscope {
  GtkWidget *drawing_area;
  GDataInputStream *input;
  int width,height;
  double fx,fy;
  uint16_t *show_data;
  uint16_t *write_data;
  uint16_t data1[NPOINTS];
  uint16_t data2[NPOINTS];
};

oscilloscope oscilloscope_new() {
  oscilloscope toReturn=malloc(sizeof(struct oscilloscope));
  memset(toReturn->data1,sizeof(toReturn->data1),0);
  memset(toReturn->data2,sizeof(toReturn->data2),0);
  toReturn->show_data=toReturn->data1;
  toReturn->write_data=toReturn->data2;
  return toReturn;
}

static void line_complete(GObject *source_object,GAsyncResult *res,gpointer user_data) {
  oscilloscope o=(oscilloscope) user_data;
  GDataInputStream *s=G_DATA_INPUT_STREAM(source_object);
  gsize len;
  GError *e;
  char* line=g_data_input_stream_read_line_finish(s,res,&len,&e);
  if (!line) return;
  int i=0;
  while (i<NPOINTS && *line) {
    if (1!=sscanf(line,"%" SCNu16,o->write_data+i)) break;
    i++;
    line=strchr(line,',');
    if (!line) break;
    line++;
  }
  if (i>=NPOINTS) {
    uint16_t *data=o->write_data;
    o->write_data=o->show_data;
    o->show_data=data;
    gtk_widget_queue_draw_area(o->drawing_area,0,0,o->width,o->height);
  }
  g_free(line);
  g_data_input_stream_read_line_async(o->input,G_PRIORITY_DEFAULT,NULL,line_complete,o);
}

static void size_callback(GtkWidget *widget,GdkRectangle *allocation,gpointer user_data) {
  oscilloscope o=(oscilloscope) user_data;
  o->width=allocation->width;
  o->height=allocation->height;
  o->fx=allocation->width/(double) NPOINTS;
  o->fy=allocation->height/(double) 4096;
}

static gboolean draw_callback (GtkWidget *widget, cairo_t *cr, gpointer data)
{
  GdkRGBA color;
  GtkStyleContext *context;
  oscilloscope o=(oscilloscope) data;
  context = gtk_widget_get_style_context (widget);

  gtk_render_background (context, cr, 0, 0, o->width, o->height);

  gtk_style_context_get_color(context,
                              gtk_style_context_get_state (context),
                              &color);
  gdk_cairo_set_source_rgba (cr, &color);
  for (int i=0; i<NPOINTS; i++) {
    double x=i*o->fx;
    double y=o->height-o->fy*o->show_data[i];
    if (i)
      cairo_line_to(cr,x,y);
    else
      cairo_move_to(cr,x,y);
  }
  cairo_stroke(cr);
  return FALSE;
}

static void
activate (GtkApplication *app,
          gpointer        user_data)
{
  GtkWidget *window;
  oscilloscope o=(oscilloscope) user_data;
  
  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "Perimeter signal");
  gtk_window_set_default_size (GTK_WINDOW (window), 1500, 800);


  o->drawing_area = gtk_drawing_area_new ();
  //gtk_widget_set_size_request (drawing_area, 1500,800);
  g_signal_connect(G_OBJECT(o->drawing_area), "size-allocate",G_CALLBACK(size_callback),o);
  g_signal_connect (G_OBJECT(o->drawing_area), "draw", G_CALLBACK (draw_callback),o);
  gtk_container_add (GTK_CONTAINER (window), o->drawing_area);
  
  gtk_widget_show_all (window);
}

gint handle_local_options(GApplication *application,GVariantDict *options,gpointer user_data)
{
  oscilloscope o=(oscilloscope) user_data;
  if (g_variant_dict_lookup(options,"version","b",NULL)) {
    g_print("oscilloscope-1.0\n");
    return 0;
  }
  gchar *device="/dev/ttyAMA1";
  g_variant_dict_lookup(options,"device","s",&device);
  int fd=open(device,O_RDONLY);
  if (fd<0) {
    g_print("Cannot open device %s\n",device);
    return 0;
  }
  struct termios mytermios;
  if (!tcgetattr(fd,&mytermios)) {
    /* This is a serial device => set speed and parity */
    cfsetspeed(&mytermios,B115200);
    cfmakeraw(&mytermios);
    mytermios.c_cflag&=~(CSTOPB);
    tcsetattr(fd,TCSANOW,&mytermios);
  }
  
  GInputStream *s=g_unix_input_stream_new(fd,TRUE);
  o->input=g_data_input_stream_new(s);
  g_data_input_stream_read_line_async(o->input,G_PRIORITY_DEFAULT,NULL,line_complete,o);
  return -1;
}

int main (int argc,char **argv)
{
  GtkApplication *app;
  int status;

  oscilloscope myoscilloscope=oscilloscope_new();

  app = gtk_application_new ("de.pfelzer.oscilloscope", G_APPLICATION_FLAGS_NONE);
  g_application_add_main_option(G_APPLICATION(app),"version",'v',G_OPTION_FLAG_NONE,G_OPTION_ARG_NONE,
                                "Show the application version", NULL);
  g_application_add_main_option(G_APPLICATION(app),"device",'d',G_OPTION_FLAG_NONE,G_OPTION_ARG_STRING,
                                "Read data from this file", NULL);
  g_application_set_option_context_summary (G_APPLICATION(app),"Display analog data");
  g_signal_connect (app, "activate", G_CALLBACK (activate),myoscilloscope);
  g_signal_connect (app, "handle-local-options",G_CALLBACK (handle_local_options),myoscilloscope);
  status = g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);

  return status;
}
