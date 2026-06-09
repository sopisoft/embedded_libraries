#[doc(hidden)]
pub trait RegisterInterface {
    type Error;

    fn read_register(&mut self, register: u8) -> Result<u8, Self::Error>;

    fn read_many(&mut self, start_register: u8, buffer: &mut [u8]) -> Result<(), Self::Error>;

    fn write_register(&mut self, register: u8, value: u8) -> Result<(), Self::Error>;

    fn write_many(&mut self, start_register: u8, values: &[u8]) -> Result<(), Self::Error>;
}
