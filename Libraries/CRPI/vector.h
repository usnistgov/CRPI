#ifndef vector_h_
#define vector_h_

struct numVector
{
  double *data;
  
  int vals;

  numVector ()
  {
    vals = 0;
    data = NULL;
  }

  ~numVector ()
  {
    if (data != NULL)
    {
      delete [] data;
      data = NULL;
      vals = 0;
    }
  }

  numVector (int len)
  {
    int y;
    vals = len;

    data = new double[vals];
    for (y = 0; y < vals; ++y)
    {
      data[y] = 0.0f;
    }
  }

  void resize (int len)
  {
    if (data != NULL)
    {
      delete [] data;
    }
    int y;
    vals = len;
    data = new double[vals];
    for (y = 0; y < vals; ++y)
    {
      data[y] = 0.0f;
    }
  }

  double& at(int pos)
  {
    if (pos < vals)
    {
      return data[pos];
    }
    return data[0];
  }

  double dotProduct (numVector &val1, numVector &val2)
  {
    if (val1.vals != val2.vals)
    {
      return 0.0f;
    }
    int y; 
    double out = 0.0f;

    for (y = 0; y < val1.vals; ++y)
    {
      out += (val1.data[y] * val2.data[y]);
    }
    return out;
  }

  bool crossProduct (numVector &val1, numVector &val2, numVector &out)
  {
    //! Currently only supports 3D vectors
    if (val1.vals != 3 || val2.vals != 3)
    {
      return false;
    }
    out.data[0] = (val1.data[1] * val2.data[2]) - (val1.data[2] * val2.data[1]);
    out.data[1] = (val1.data[2] * val2.data[0]) - (val1.data[0] * val2.data[2]);
    out.data[2] = (val1.data[0] * val2.data[1]) - (val1.data[1] * val2.data[0]);
    return true;
  }
};


struct vector3D
{
  double i;
  double j;
  double k;

  vector3D ()
  {
    i = j = k = 0.0f;
  }

  ~vector3D ()
  {
  }

  double& at(int pos)
  {
    if (pos == 0)
    {
      return i;
    }
    else if (pos == 1)
    {
      return j;
    }
    else
    {
      return k;
    }
  }

  double dotProduct (vector3D &val1, vector3D &val2)
  {
    double out = (val1.i * val2.i) + (val1.j * val2.j) + (val1.k * val2.k);
    return out;
  }

  bool crossProduct (vector3D &val1, vector3D &val2, vector3D &out)
  {
    out.i = -((val1.j * val2.k) - (val1.k * val2.j));
    out.j = -((val1.k * val2.i) - (val1.i * val2.k));
    out.k = -((val1.i * val2.j) - (val1.j * val2.i));
    return true;
  }
};

#endif