package pomdp.utilities;

public class JProf {

	protected static boolean m_bEnabled = false;

	private static native long getCurrentThreadCpuTime();

	public static long getCurrentThreadCpuTimeSafe(){
		if( m_bEnabled ){
			long iTime = getCurrentThreadCpuTime();
			return iTime;
		}
		else{
			return System.nanoTime();
		}
	}

	static{
		try{
			System.loadLibrary( "capjprof" );
			System.out.println( "capjprof loaded" );
			m_bEnabled = true;
		}
		catch( UnsatisfiedLinkError err ){
			m_bEnabled = false;
			//System.out.println( "unable to load capjprof " + err );
		}
	}
}
