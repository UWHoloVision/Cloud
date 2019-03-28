using System.Threading;
using UnityEngine;

public class MeshBehaviour : MonoBehaviour
{

#if ENABLE_WINMD_SUPPORT
    MeshSocket meshSocket;
    long MeshUpdate = 0;
    GameObject cube;

    public async void Start()
    {
        cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.position = Vector3.zero;
        cube.GetComponent<MeshFilter>().mesh.Clear();
        meshSocket = await MeshSocket.CreateAsync();
    }

    public void Update()
    {
        var last = Interlocked.Read(ref meshSocket.MeshId);
        var cmp = Interlocked.Exchange(ref MeshUpdate, last);
        if (last != cmp) // mesh has updated!
        {
            Debug.Log("Rendering mesh");
            Mesh mesh = new Mesh();
            mesh.vertices = meshSocket.vertices;
            mesh.triangles = meshSocket.triangles;
            mesh.RecalculateNormals();
            cube.GetComponent<MeshFilter>().mesh = mesh;
            cube.transform.position = Vector3.zero;
            cube.transform.rotation = Quaternion.Euler(Vector3.zero);
            cube.transform.localScale = Vector3.one; // transformed on server-side from rhs->lhs
            cube.GetComponent<Renderer>().material = new Material(Shader.Find("Custom/MeshShader"));
            cube.GetComponent<Renderer>().material.color = Color.red;
            Debug.Log("Rendered mesh");
        }
    }
#endif // ENABLE_WINMD_SUPPORT
}
