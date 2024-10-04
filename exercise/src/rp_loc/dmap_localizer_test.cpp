#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sys/time.h>
#include "dmap_localizer.h"
#include "rp_base/draw_helpers.h"

using namespace std;
using Eigen::Isometry2f;
using Eigen::Rotation2Df;

int main(int argc, char** argv) {
  if (argc<5) {
    std::cout << "usage " << argv[0] << " <num_obstacles> <range> <resolution> <dmax>" << std::endl;
    return -1;
  }

  int num_obstacles=atoi(argv[1]);
  float range=atof(argv[2]);
  float resolution = atof(argv[3]);
  float dmax = atof(argv[4]);
  cerr << "parameters: " << endl;
  cerr << " obstacles: " << num_obstacles << endl;
  cerr << " range: " << range << endl;
  cerr << " resolution: " << resolution << endl;
  cerr << " dmax_influence: " << dmax << endl;

  // TODO #1: generate random obstacles
  cerr << "obstacles" << endl;

  // TODO #2:
  // - construct the localizer using the obstacles
  // - compute the distance map (call the setMap method)
  cerr << "localizer ready" << endl;
  cerr << "rows:  " << localizer.distances.rows << " cols: " << localizer.distances.cols << endl;

  // prepare canvas for visualization
  Canvas canvas;
  const auto& distances = localizer.distances;
  Grid_<uint8_t> image(distances.rows, distances.cols);
  
  // compute normalization of the DMAP
  float f_min=1e9;
  float f_max=0;
  for(auto& f: distances.cells) {
    f_min=std::min(f, f_min);
    f_max=std::max(f, f_max);
  }
  float scale=255./(f_max-f_min);

  // copy the (normalized) distances of the DMAP
  for (size_t i=0; i<distances.cells.size(); ++i) {
    image.cells[i]=scale  * (distances.cells[i] - f_min);
  }

  // TODO #4: compute normalization of the ROW DERIVATIVE
  // TODO #5: copy the (normalized) distances of the ROW DERIVATIVE

  // TODO #7: compute normalization of the COL DERIVATIVE
  // TODO #8: copy the (normalized) distances of the COL DERIVATIVE

  // TODO #11: compute normalization of the DERIVATIVE MAGNITUDE
  // TODO #12: copy the (normalized) distances of the DERIVATIVE MAGNITUDE

  // add an obstacle image as alternative visualization
  Grid_<uint8_t> obstacle_image(distances.rows, distances.cols);
  obstacle_image.fill(0);
  drawGrid(canvas, obstacle_image);
  for (const auto& m: obstacles) {
    Vector2f m_hat_grid=localizer.gm.world2grid(m);
    drawCircle(canvas, m_hat_grid.cast<int>(), 3, 255);
  }
  // we draw with cv and get back the result, dirty...
  memcpy(&obstacle_image.cells[0], canvas.data, distances.rows*distances.cols);

  // initialize the localizer with a small offset
  Isometry2f X=Eigen::Isometry2f::Identity();
  X.linear()=Rotation2Df(0.3).matrix();
  X.translation()<< 1, 0.5;
  localizer.X=X;

  int current_key = 0;
  while (1) {
    switch (current_key) {
      case 0:
        drawGrid(canvas, image);
        break;
      case 1:
        drawGrid(canvas, obstacle_image);
        break;
      // TODO #6: case for ROW DERIVATIVE
      // TODO #9: case for COL DERIVATIVE
      // TODO #13: case for DERIVATIVE MAGNITUDE
      default:;
    }

    for (const auto& m: obstacles) {
      Vector2f m_hat=localizer.X*m;
      Vector2f m_hat_grid=localizer.gm.world2grid(m_hat);
      drawCircle(canvas, m_hat_grid.cast<int>(), 3, 127);
    }
    int key = showCanvas(canvas,0);
    if (key == 32) {
      current_key = (current_key + 1) % 2;
      continue;
    }

    struct timeval tv_start, tv_end, tv_delta;
    gettimeofday(&tv_start,0);
    // TODO #3: use the localizer to perform one localization step
    gettimeofday(&tv_end,0);
    timersub(&tv_end, &tv_start, &tv_delta);
    cout << "time: " << tv_delta.tv_sec*1e3 + tv_delta.tv_usec*1e-3 << endl;
  }
}
