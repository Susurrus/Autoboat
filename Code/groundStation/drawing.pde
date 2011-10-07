// Renders a vector object 'v' as an arrow at a location xy.
// The expected coordinate system should match Processing's
// ESD coordinate system.
// From: http://www.shiffman.net/2011/02/03/rotate-a-vector-processing-js/
void drawVector(PVector v, float x, float y, float scayl, float weight, PVector rgb) {
  pushMatrix();
  float arrowsize = 10;
  // Translate to location to render vector
  translate(x,y);
  
  // Set a stroke weight
  if (weight > 0) {
    strokeWeight(weight);
  }
  
  // Set a vector color
  if (rgb.mag() == 0) {
    stroke(255);
  } else {
    stroke(rgb.x, rgb.y, rgb.z);
  }
  // Vector heading to get direction (pointing up is a heading of 0)
  rotate(v.heading2D());
  // Scale it to be bigger or smaller if necessary
  float len = v.mag()*scayl;
  // Draw three lines to make an arrow
  line(0,0,len,0);
  line(len,0,len-arrowsize,+arrowsize/2);
  line(len,0,len-arrowsize,-arrowsize/2);
  popMatrix();
}
