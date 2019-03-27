using System.Threading;
using UnityEngine;
using System.Linq;

public class MeshBehaviour : MonoBehaviour
{

#if ENABLE_WINMD_SUPPORT
    MeshSocket meshSocket;
    long MeshUpdate = 0;
    GameObject cube;

    public async void Start()
    {
        Debug.Log("Mesh rendering");
        cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        // cube.GetComponent<MeshFilter>().mesh.Clear();
        cube.GetComponent<MeshFilter>().mesh.vertices = new Vector3[]
        {
            new Vector3(0, 0, 0),
            new Vector3(10, 0, 0),
            new Vector3(0, 10, 0)
        };
        cube.GetComponent<MeshFilter>().mesh.triangles = new int[] { 0, 1, 2 };
        // should do inversetransform for pts
        // cube.transform.position = Camera.main.transform.position + Camera.main.transform.forward;
        cube.transform.position = Camera.main.transform.position;
        // cube.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
        cube.GetComponent<MeshFilter>();
        meshSocket = await MeshSocket.CreateAsync();
    }

    public void Update()
    {
        var last = Interlocked.Read(ref meshSocket.MeshId);
        var cmp = Interlocked.Exchange(ref MeshUpdate, last);
        if (last != cmp) // mesh has updated!
        {
            cube.GetComponent<MeshFilter>().mesh.vertices = 
                meshSocket.vertices.Select(v => Camera.main.transform.InverseTransformPoint(v))
                    .ToArray();
            cube.GetComponent<MeshFilter>().mesh.triangles =
                meshSocket.triangles;
            cube.transform.localPosition = Vector3.zero;
            cube.transform.localRotation = Quaternion.identity;
        }
    }
#endif // ENABLE_WINMD_SUPPORT
}
